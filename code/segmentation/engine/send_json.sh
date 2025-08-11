URL=$1
FILE=$2

curl -vsS -H "Content-Type: application/json" \
  --data-binary @"$FILE" \
  "$URL"
