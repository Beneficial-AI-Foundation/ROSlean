#!/usr/bin/env bash
# Prepares a fresh Linux machine (a Claude Code cloud session) for this
# repository: elan (Lean), pixi (ROS 2 Jazzy from RoboStack), the Mathlib
# cache and the pixi environment.
#
#   scripts/cloud_setup.sh            # from anywhere inside the checkout
#
# Safe to run twice: every step first checks whether it is already done and
# says so. The script cannot change the PATH of the shell that called it, so
# afterwards run, in every new shell:
#
#   export PATH="$HOME/.elan/bin:$HOME/.pixi/bin:$PATH"
#
# Steps can be skipped: SKIP_MATHLIB_CACHE=1, SKIP_PIXI_ENV=1.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

say() { printf '[cloud_setup] %s\n' "$*"; }
start=$SECONDS
step_start=$SECONDS
took() { say "  took $((SECONDS - step_start)) s"; step_start=$SECONDS; }

export PATH="$HOME/.elan/bin:$HOME/.pixi/bin:$PATH"

say "machine: $(uname -m), $(. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME:-unknown OS}"), $(nproc) cores, $(awk '/MemTotal/ {printf "%.0f GB RAM", $2/1048576}' /proc/meminfo)"

# --- elan (Lean toolchain manager) ------------------------------------------
if command -v elan >/dev/null 2>&1; then
  say "elan: already installed ($(elan --version))"
else
  say "elan: installing into ~/.elan"
  curl -sSf https://raw.githubusercontent.com/leanprover/elan/master/elan-init.sh \
    | sh -s -- -y --default-toolchain none
  say "elan: installed ($(elan --version))"
fi
took

# --- Lean toolchain named in lean-toolchain ---------------------------------
# elan downloads from releases.lean-lang.org. Where the network policy blocks
# that host, the same release is taken from github.com and unpacked into the
# directory elan would have used.
toolchain="$(tr -d '[:space:]' < lean-toolchain)"   # leanprover/lean4:v4.37.0-rc1
version="${toolchain##*:}"                           # v4.37.0-rc1
tc_dir="$HOME/.elan/toolchains/$(printf '%s' "$toolchain" | sed 's|/|--|; s|:|---|')"
if [ -x "$tc_dir/bin/lake" ]; then
  say "lean: toolchain $toolchain already installed"
elif elan toolchain install "$toolchain" >/dev/null 2>&1; then
  say "lean: toolchain $toolchain installed by elan"
else
  say "lean: elan could not download $toolchain (releases.lean-lang.org unreachable?)"
  case "$(uname -m)" in
    x86_64) lean_arch=linux ;;
    aarch64) lean_arch=linux_aarch64 ;;
    *) say "lean: no fallback for $(uname -m)"; exit 1 ;;
  esac
  name="lean-${version#v}-$lean_arch"
  url="https://github.com/leanprover/lean4/releases/download/$version/$name.zip"
  say "lean: falling back to $url (about 1 GB)"
  command -v unzip >/dev/null 2>&1 || { say "lean: unzip is needed for the fallback"; exit 1; }
  tmp="$(mktemp -d "$HOME/.elan/toolchains/.dl.XXXXXX")"
  trap 'rm -rf "$tmp"' EXIT
  curl -fsSL -o "$tmp/lean.zip" "$url"
  unzip -q "$tmp/lean.zip" -d "$tmp"
  rm -rf "$tc_dir"
  mv "$tmp/$name" "$tc_dir"
  rm -rf "$tmp"; trap - EXIT
  say "lean: toolchain $toolchain installed from github.com"
fi
say "lean: $(lake --version)"
took

# --- pixi -------------------------------------------------------------------
if command -v pixi >/dev/null 2>&1; then
  say "pixi: already installed ($(pixi --version))"
else
  say "pixi: installing into ~/.pixi/bin"
  if curl -fsSL --max-time 30 https://pixi.sh/install.sh -o /dev/null 2>/dev/null; then
    curl -fsSL https://pixi.sh/install.sh | PIXI_NO_PATH_UPDATE=1 sh
  else
    say "pixi: pixi.sh unreachable, taking the release from github.com"
    mkdir -p "$HOME/.pixi/bin"
    curl -fsSL "https://github.com/prefix-dev/pixi/releases/latest/download/pixi-$(uname -m)-unknown-linux-musl.tar.gz" \
      | tar -xz -C "$HOME/.pixi/bin"
  fi
  say "pixi: installed ($(pixi --version))"
fi
took

# --- Mathlib cache ----------------------------------------------------------
# `lake exe cache get` fetches about 9000 files from cache.mathlib.org. Where
# that host is blocked every single file fails after some seconds and the
# command runs for hours, so the host is probed first.
if [ "${SKIP_MATHLIB_CACHE:-0}" = 1 ]; then
  say "mathlib cache: skipped (SKIP_MATHLIB_CACHE=1)"
elif curl -sS -o /dev/null --max-time 20 https://cache.mathlib.org/ 2>/dev/null; then
  say "mathlib cache: lake exe cache get (downloads only what is missing)"
  lake exe cache get
  say "mathlib cache: done"
else
  say "mathlib cache: cache.mathlib.org is unreachable from here; NOT downloaded."
  say "mathlib cache: fetching the Lean dependencies' sources instead (git, github.com)"
  # Clones the packages of lake-manifest.json; changes nothing if present.
  lake env true >/dev/null
  say "mathlib cache: 'lake build' will compile the needed part of Mathlib from source (slow, once)."
fi
took

# --- pixi environment (ROS 2 Jazzy, Gazebo, Nav2) ---------------------------
if [ "${SKIP_PIXI_ENV:-0}" = 1 ]; then
  say "pixi environment: skipped (SKIP_PIXI_ENV=1)"
else
  if [ -d .pixi/envs/default/conda-meta ]; then
    say "pixi environment: present, checking it against pixi.lock"
  else
    say "pixi environment: installing from pixi.lock (several GB)"
  fi
  pixi install --locked
  say "pixi environment: ready ($(du -sh .pixi/envs/default 2>/dev/null | cut -f1))"
fi
took

say "done in $((SECONDS - start)) s. In every new shell:"
say '  export PATH="$HOME/.elan/bin:$HOME/.pixi/bin:$PATH"'
