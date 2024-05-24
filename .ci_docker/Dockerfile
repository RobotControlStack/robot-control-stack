# syntax=docker/dockerfile:1
FROM ubuntu:latest

# dependencies for our repo
RUN apt-get update && apt-get install --yes build-essential cmake git libpoco-dev libeigen3-dev libxslt-dev libcoin-dev libccd-dev libglfw3-dev libboost-all-dev liblzma-dev ninja-build clang python-is-python3 python3-venv python3-pip
# dependencies for linting
RUN apt-get update && apt-get install --yes clang-tidy clang-format

# dependencies for gitlab-runner
WORKDIR /rcs
COPY script.deb.sh .
RUN ./script.deb.sh
RUN rm ./script.deb.sh
RUN apt-get install --yes gitlab-runner
ARG TOKEN
RUN gitlab-runner register --url https://gitos.rrze.fau.de --non-interactive --executor shell --token ${TOKEN}
CMD gitlab-runner run
