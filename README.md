# Robotics Software Engineer Technical Assessment
This project is  technical assessment for a position in Progressive Robotics. It is divided in three main steps.

## Step 1: Development Enviroment with Docker
The first step is to set up the development enviroment required to run the ROS2 software pipeline. Since ROS2 Humble is mainly for Ubuntu 22.04, and teh project needs to remain portable across different operating systems, Docker is used to provide a consistent and reproducible development enviroment.. 

### Docker
Docker is a containerization software that creates an isolated enviroment called container, packaging all dependencies required by the application.
To begin, Docker Desktop must be installed
// the docker application if we don't already have it installed "https://www.docker.com/products/docker-desktop/".

After installing the app, we need to create two files in the directory. 
The first file is Dockerfile and it contains all the comands that we would run inside the Ubuntu OS.

The second file is the docker-compose.yml and it acts as the file that will mount the directories inside the virtual os to an actual dir in the system.
