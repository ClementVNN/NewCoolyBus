# Docker Commands


## IMAGES
- Downloading Images
```
docker pull <image name>
```
- List all images
```
docker images -a
```
- Delete single Image
```
docker rmi <image id>
```
- All images delete
```
docker rmi $(docker images  -q)
```


## CONTAINERS
- Creating a interactive container  from image
```
docker run -it <image name>
```
- Giving Name to a container while creating
```
docker run --name <container name> <image name>
```
- Start a stopped Container
```
docker start (container_id)
```
- Stop all containers
```
sudo docker kill $(sudo docker ps -a)
```
- Connect shell to running container
```
docker exec -it (container_id) bash
Get running container id via : docker ps -ql

```
- Delete single Container
```
docker rm <container id or container name>
```
- Delete all containers
```
docker rm $(docker ps -a -q)
```
## Building Image from Docker File
- Terminal from same directory
```
docker built -t <image name > .
```

