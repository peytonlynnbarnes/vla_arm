#!/usr/bin/env bash
# Run Claude Code inside a containerized ROS 2 Jazzy workspace.
#
# Bypass-permissions is safe-er here because the container:
#   - only mounts /workspace (NOT $HOME, NOT ~/.ssh, ~/.aws, ~/.config/gh)
#   - runs as a non-root user with your UID (edits land with correct ownership)
#   - keeps Claude auth in ~/.claude-vla-arm on the host, separate from your
#     real ~/.claude so the container can't read or overwrite it
#
# Host localhost:8000 (the VLA SSH tunnel) is reachable via --network=host.
# That also means the container can reach anything else on your host's
# loopback — acceptable trade-off for Jazzy/ROS_DOMAIN_ID compatibility.
#
# Pass any extra args to claude after --, e.g.:
#   ./run.sh -- --model opus
# Or drop into a shell instead of claude:
#   ./run.sh shell

set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=vla-arm-claude:latest
CLAUDE_HOME="$HOME/.claude-vla-arm"

# Refuse to mount a protected branch — bypass mode gets its own playground.
# Override with CLAUDE_ALLOW_PROTECTED=1 if you really mean it.
branch="$(git -C "$REPO" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '')"
case "$branch" in
    main|master|jazzy)
        if [ "${CLAUDE_ALLOW_PROTECTED:-0}" != "1" ]; then
            echo "Refusing to run bypass-mode container on branch '$branch'." >&2
            echo "Run from a worktree instead:" >&2
            echo "  git worktree add ../$(basename "$REPO")-claude -b claude/playground" >&2
            echo "  cd ../$(basename "$REPO")-claude && ./.devcontainer/run.sh" >&2
            echo "Or set CLAUDE_ALLOW_PROTECTED=1 to override." >&2
            exit 1
        fi
        ;;
esac

mkdir -p "$CLAUDE_HOME"

docker build \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t "$IMAGE" \
    "$REPO/.devcontainer"

COMMON_ARGS=(
    --rm -it
    --name vla-arm-claude
    --network=host
    -v "$REPO:/workspace"
    -v "$CLAUDE_HOME:/home/dev/.claude"
    -w /workspace
)

if [ "${1:-}" = "shell" ]; then
    exec docker run "${COMMON_ARGS[@]}" "$IMAGE" bash
fi

if [ "${1:-}" = "--" ]; then
    shift
fi

exec docker run "${COMMON_ARGS[@]}" "$IMAGE" \
    claude --permission-mode bypassPermissions "$@"
