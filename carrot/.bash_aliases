
#
# bash aliases
#

alias so='source $PROJ_ROOT/.mosa-init'
alias h='history'

# handy command
function cd() { builtin cd "$@" && pwd; }
function ff() { find . -name "$@" -print; }

# end of file
