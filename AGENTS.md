# Workspace Rules

## WSL and local workspace synchronization

- The local repository is the source of truth. Do not make independent source
  edits in `/root/smartcar2026-simulation`.
- Make every source change in the local repository, commit and push it, then
  fast-forward `/root/smartcar2026-simulation` to the same commit.
- Before reporting a change as deployed, verify that the local repository and
  the root WSL workspace have the same `HEAD` commit and that both worktrees
  are clean.
- If the root WSL workspace contains uncommitted changes, do not overwrite it.
  Reconcile those changes into the local repository first, then resume sync.
- Do not synchronize generated `build/`, `devel/`, `log/`, `.ros/`, or Gazebo
  cache files; build artifacts remain local to the WSL deployment.

## Global Rules

- Do not use Playwright MCP. When browser automation or Playwright testing is
  needed, use the Playwright CLI instead.
