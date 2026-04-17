# Robotics Software Engineer Technical Assessment
This project is a technical assessment for a position in Progressive Robotics. It is divided in three main steps.

## Step 1: Development Environment with Docker
The first step is to set up the development environment required to run the ROS2 software pipeline. Since ROS2 Humble is mainly for Ubuntu 22.04, and the project needs to remain portable across different operating systems, Docker is used to provide a consistent and reproducible development environment.

### Docker
Docker is a containerization software that creates an isolated environment called container, packaging all dependencies required by the application. To begin, Docker Desktop, must be installed if not already in the system, the link: "https://www.docker.com/products/docker-desktop/". After installing the app, two necessary files are needed. 

The first file is the Dockerfile and it contains commands for building the ready to run system with the preferable OS and all the packages needed to run the project from anywhere.

The second file is the docker-compose.yml, the yml file acts as the orchestration of the system. It specifies what images,  are required, ports that need to be exposed, communication and acces to the host systems, such as file system, and so on.

### How to Run
After writing the two files for the system, the build command builds the image:

```bash
docker compose build
```

After successfully building the image, in order to start the container and run to the background:

```bash
docker compose up -d
```
To verify the container is running:
```bash
docker ps
```
Last necessary command that launches a shell inside the container:
```bash
docker exec -it <container_name> /bin/bash
```

### Troubleshooting
Because the Docker builds an image on Windows, it runs inside a lightweight Linux VM managed by Docker Desktop. That VM sometimes uses Windows' DNS setting, which can be misconfigured through a different DNS resulting to random 'Bad Request' errors when trying to download packages. The fix was to explicitly tell Docker to use a public DNS server instead of the system default.
In the system settings->Docker Engine of Docker Desktop:
```json
  "dns": [
    "8.8.8.8",
    "1.1.1.1"
  ],
```
This network issue was diagnosed with the assistance of an LLM during debugging.


