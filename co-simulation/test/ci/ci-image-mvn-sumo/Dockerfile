FROM maven:3.8.3-adoptopenjdk-11

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --allow-unauthenticated software-properties-common && \
    add-apt-repository ppa:sumo/stable && \
    apt-get install -y sumo