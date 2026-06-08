# ~/.zsh_prompt.zsh
# Source this file from .zshrc:  source ~/.zsh_prompt.zsh

setopt PROMPT_SUBST
autoload -Uz add-zsh-hook

# ── Icons ─────────────────────────────────────────────────────────────────────
GIT_ICON=$''   # nf-dev-git        U+E702
KUBE_ICON=$''  # nf-dev-kubernetes U+E81D
TIME_ICON=$''  # nf-fa-clock       U+F017

# ── Last command execution time ───────────────────────────────────────────────
__cmd_start_time=0

__preexec_timer() {
    __cmd_start_time=$SECONDS
}

__cmd_duration() {
    [[ $__cmd_start_time -eq 0 ]] && return
    local duration=$(( SECONDS - __cmd_start_time ))
    __cmd_start_time=0
    [[ $duration -lt 1 ]] && return

    if [[ $duration -ge 3600 ]]; then
        print -P "%F{red}󰔛 $(( duration / 3600 ))h$(( (duration % 3600) / 60 ))m$(( duration % 60 ))s%f"
    elif [[ $duration -ge 60 ]]; then
        print -P "%F{yellow}󰔛 $(( duration / 60 ))m$(( duration % 60 ))s%f"
    else
        print -P "%F{green}󰔛 ${duration}s%f"
    fi
}

add-zsh-hook preexec __preexec_timer

# ── Git info ──────────────────────────────────────────────────────────────────
__git_info() {
    local branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
    [[ -z "$branch" ]] && return

    local commit=$(git rev-parse --short HEAD 2>/dev/null)
    local dirty=$(git status --porcelain 2>/dev/null | wc -l | tr -d ' ')

    if [[ "$dirty" -gt 0 ]]; then
        print -P "%F{magenta}${GIT_ICON} $branch%f%F{gray} @ %f%F{cyan}$commit %F{yellow}✗%f"
    else
        print -P "%F{magenta}${GIT_ICON} $branch%f%F{gray} @ %f%F{cyan}$commit %F{green}✓%f"
    fi
}

# ── Kubernetes info ───────────────────────────────────────────────────────────
__kube_info() {
    local ctx=$(timeout 1 kubectl config current-context 2>/dev/null)
    [[ -z "$ctx" ]] && return

    local ns=$(timeout 1 kubectl config view --minify --output 'jsonpath={..namespace}' 2>/dev/null)
    ns=${ns:-default}

    local ns_color="cyan"
    [[ "$ns" == *"prod"* ]] && ns_color="red"

    print -P "%F{blue}${KUBE_ICON} $ctx %F{$ns_color}⎈ $ns%f"
}

# ── Prompt (built in precmd so icon variables expand correctly) ───────────────
__build_prompt() {
    PS1="%F{gray}╭─%f%F{blue}${TIME_ICON} %f%F{cyan}%*%f%F{gray} %f%B%F{green} %n%f%b%F{gray} in %f%B%F{yellow} %~%f%b \$(__git_info)\$(__kube_info) \$(__cmd_duration)
%F{gray}╰─%f%B%F{magenta} ❯ %f%b"
}

add-zsh-hook precmd __build_prompt
