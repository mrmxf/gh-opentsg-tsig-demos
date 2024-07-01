# 
# |_  _ .  _     _|  _  _   _   _
# |_ _) | (_)   (_| (- ||| (_) _)
#         _/
source <(clog Inc)
PROJECT=tsig-demos
bEXE="generator"
svelteFolder=""
callingSCRIPT="${0##*/}"
vCodeType="golang"
vCodeSrc="./releases.yaml"
vCODE=$(cat $vCodeSrc | grep version | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
bMSG=$(cat  $vCodeSrc | grep note    | head -1 | sed -nr "s/note: (.*)/\1/p" | xargs)

