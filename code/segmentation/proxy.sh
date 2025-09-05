docker stop quick-proxy
docker run --rm -d \
  -p 8081:8081 -p 8082:8082 -p 8083:8083 \
  -v $(pwd)/nginx.conf:/etc/nginx/nginx.conf:ro \
  --name quick-proxy nginx
