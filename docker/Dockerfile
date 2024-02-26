# syntax=docker/dockerfile:1
FROM debian:stable
# dependencies for our repo
RUN apt-get update && apt-get install --yes build-essential cmake git libpoco-dev libeigen3-dev libxslt-dev libcoin-dev libccd-dev libglfw3-dev libboost-all-dev liblzma-dev ninja-build clang
# dependencies for linting
RUN apt-get update && apt-get install --yes clang-tidy clang-format
# dependencies for gitlab-runner
WORKDIR /root
COPY script.deb.sh .
RUN ./script.deb.sh
RUN rm ./script.deb.sh
RUN apt-get install --yes gitlab-runner
ARG TOKEN
RUN gitlab-runner register --url https://gitos.rrze.fau.de --non-interactive --executor shell --token ${TOKEN}
CMD gitlab-runner run
