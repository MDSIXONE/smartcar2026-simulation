#!/usr/bin/env python3
"""Build the small HOG/SVM label guard used after YOLOv5 localisation.

YOLOv5 remains responsible for finding the cube.  This classifier uses only
the pixels inside that detected box and prevents a low-confidence class-head
mix-up from selecting the wrong pickup bay.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


def crop_feature(image, box):
    x1, y1, x2, y2 = [float(value) for value in box]
    width = x2 - x1
    height = y2 - y1
    x1 = max(0, int(round(x1 - 0.10 * width)))
    y1 = max(0, int(round(y1 - 0.08 * height)))
    x2 = min(image.shape[1], int(round(x2 + 0.10 * width)))
    y2 = min(image.shape[0], int(round(y2 + 0.08 * height)))
    crop = image[y1:y2, x1:x2]
    if crop.size == 0:
        raise ValueError("empty crop for box %s" % box)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (64, 64), interpolation=cv2.INTER_AREA)
    gray = cv2.equalizeHist(gray)
    hog = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)
    return hog.compute(gray).reshape(-1)


def yolo_best_box(image, session, input_name, output_name, input_size=640):
    image_height, image_width = image.shape[:2]
    scale = min(
        float(input_size) / float(image_width),
        float(input_size) / float(image_height),
    )
    resized_width = int(round(image_width * scale))
    resized_height = int(round(image_height * scale))
    resized = cv2.resize(
        image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
    )
    pad_x = (input_size - resized_width) // 2
    pad_y = (input_size - resized_height) // 2
    canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized
    blob = np.ascontiguousarray(
        canvas[:, :, ::-1].transpose((2, 0, 1)), dtype=np.float32
    )[np.newaxis, :] / 255.0
    rows = session.run([output_name], {input_name: blob})[0][0]
    confidences = rows[:, 4] * np.max(rows[:, 5:], axis=1)
    row = rows[int(np.argmax(confidences))]
    center_x, center_y, width, height = [float(value) for value in row[:4]]
    return [
        max(0.0, (center_x - width / 2.0 - pad_x) / scale),
        max(0.0, (center_y - height / 2.0 - pad_y) / scale),
        min(image_width - 1.0, (center_x + width / 2.0 - pad_x) / scale),
        min(image_height - 1.0, (center_y + height / 2.0 - pad_y) / scale),
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    dataset = Path(args.dataset).resolve()
    records = [
        json.loads(line)
        for line in (dataset / "metadata.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    features = []
    labels = []
    session = ort.InferenceSession(
        str(Path(args.onnx).resolve()),
        providers=["CPUExecutionProvider"],
    )
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    for record in records:
        image = cv2.imread(str(dataset / record["image"]))
        if image is None:
            raise RuntimeError("cannot read %s" % record["image"])
        detected_box = yolo_best_box(
            image, session, input_name, output_name
        )
        features.append(crop_feature(image, detected_box))
        labels.append(int(record["class_id"]))

    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setC(2.0)
    svm.train(
        np.asarray(features, dtype=np.float32),
        cv2.ml.ROW_SAMPLE,
        np.asarray(labels, dtype=np.int32),
    )
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(output))
    print("saved %d visual label samples to %s" % (len(records), output))


if __name__ == "__main__":
    main()
