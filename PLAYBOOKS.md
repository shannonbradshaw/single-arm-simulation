# Playbooks

Checklists for common operations. Follow these step-by-step — do not skip verification steps or claim completion without evidence.

## Deploy Changes to VM

Use this whenever any file that goes into the Docker image has been modified locally.

### 1. Identify changed files

```bash
git diff --name-only   # staged + unstaged
git diff --name-only --cached  # staged only
```

Know exactly which files need to reach the VM.

### 2. Copy files to VM build directory

```bash
gcloud compute scp --tunnel-through-iap --zone us-central1-a \
  <local-file> shannon-bradshaw@xarm6-sim:~/<filename>
```

Then move into the build directory (different user owns it):

```bash
gcloud compute ssh shannon-bradshaw@xarm6-sim --zone us-central1-a --tunnel-through-iap \
  --command "sudo cp ~/filename /home/shannon.bradshaw/single-arm-simulation/<path>"
```

### 3. Verify file hashes match

Compare local hash:
```bash
md5 -q <local-file>
```

Against VM build directory hash:
```bash
gcloud compute ssh ... --command "sudo md5sum /home/shannon.bradshaw/single-arm-simulation/<path>"
```

**Stop here if hashes don't match.** Diagnose and re-copy.

### 4. Rebuild Docker image

```bash
gcloud compute ssh ... --command "sudo bash -c 'cd /home/shannon.bradshaw/single-arm-simulation && docker build -f Dockerfile.gpu -t xarm6-sim-gpu:latest . 2>&1 | tail -20'"
```

Confirm the build completes with `naming to docker.io/library/xarm6-sim-gpu:latest done`.

### 5. Restart container

```bash
gcloud compute ssh ... --command "sudo docker rm -f xarm6-full && sudo docker run -d --name xarm6-full --runtime=nvidia --gpus all -p 502:502 -p 8080:8080 -p 8081:8081 -e VIAM_CONFIG=/opt/viam_config_cloud.json xarm6-sim-gpu:latest"
```

### 6. Verify file inside running container

```bash
gcloud compute ssh ... --command "sudo docker exec xarm6-full md5sum /opt/<path>"
```

**Hash must match the local file.** If it doesn't, the Dockerfile COPY is wrong or the build context was stale.

### 7. Check container health

```bash
gcloud compute ssh ... --command "sudo docker logs xarm6-full 2>&1 | tail -30"
```

Confirm all of these appear in the logs:
- `[xarm-emu] Listening on 0.0.0.0:502` — emulator started
- `Web viewer starting on http://0.0.0.0:8081` — web viewer started
- `serving` with URL — viam-server started
- `[gripper_mimic] Configured 5 mimic joints` — gripper plugin loaded
- No Python tracebacks or crash loops

### 8. Functional verification

- Open web viewer at `http://34.30.31.198:8081` — confirm rendering looks correct
- If arm changes: use Viam app Control tab to move joints, verify they respond
- If gripper changes: use Open/Grab test cards, verify symmetric closure
- If visual changes: visually confirm in web viewer before reporting done

**Do not report success until steps 6-8 pass.**

---

## Commit and Push

### 1. Review changes

```bash
git status
git diff
```

Read what's actually changed. Don't commit files you haven't reviewed.

### 2. Stage specific files

```bash
git add <file1> <file2> ...
```

Never use `git add -A` or `git add .` — review each file.

### 3. Write commit message

Summarize the "why", not the "what". End with co-author line.

### 4. Push

Only push when explicitly asked.

---

## Diagnose Arm/Gripper Issues

### 1. Check joint positions via emulator protocol

Use the diagnostic script pattern from `/tmp/test_gripper_diag.py` — connects to port 502, sends Modbus commands, reads joint states from gz-transport, and traces positions over time.

### 2. Check Gazebo logs

```bash
sudo docker logs xarm6-full 2>&1 | grep -E "gripper_mimic|Err|WARN|error"
```

### 3. Check joint state topic

Inside the container:
```bash
gz topic -e -t /xarm6/joint_states -n 1
```

### 4. Key invariants

- Arm home position: [0, -45, -30, 0, 60, 0] degrees (J1-J6)
- Gripper protocol: 0=closed, 850=open
- Gazebo gripper: 0=open, 0.85rad=closed
- drive_joint controls left outer knuckle; 5 mimic joints follow via gripper_mimic.py
- J4-J6 use velocity command mode (not PID force)
- Arm collision bitmask is 0x00

---

## Add a New File to the Docker Image

1. Create/modify the file locally
2. Add a `COPY` line in `Dockerfile.gpu`
3. If it's a Python module, ensure `PYTHONPATH=/opt` covers it
4. If it's an executable, add `chmod +x` in the Dockerfile
5. Follow the full "Deploy Changes to VM" playbook above — including copying `Dockerfile.gpu` itself
