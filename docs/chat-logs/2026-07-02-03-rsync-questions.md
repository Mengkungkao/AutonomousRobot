# Session 3 — rsync questions

**Session ID:** `6d40f038-d4e5-48c3-b7d9-6e4343f4c6d2`
**Time:** 14:00:17 – 14:01:54 local

Advisory session, no repo changes.

## Q1: How to rsync files to another machine without deleting/re-cloning the repo?
Answer: use `rsync -avz --exclude='.git/' /path/to/local/repo/ user@remote-host:/path/to/remote/repo/` — no `--delete` flag, so files present remotely but not locally are left alone; `--exclude='.git/'` protects the remote's own git history.

## Q2: How to rsync to bring a machine to the newest `main`?
Answer: if the remote already has its own git clone (as in this project's case, e.g. the Pi), just `git pull` there — no rsync needed:
```bash
ssh user@remote-host "cd ~/AutonomousRobot && git fetch origin && git checkout main && git pull"
```
rsync (with `--delete --exclude='.git/' --filter=':- .gitignore'`) was suggested only as a fallback for a remote with no git/network access to the origin, syncing from a clean local `main` checkout instead.
