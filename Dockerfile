ARG BASE_IMAGE=rust:1.67-slim-bullseye

FROM $BASE_IMAGE AS prereqs

ENV CARGO_HOME=/cargo_home

RUN apt update && apt install -y pkg-config libusb-1.0-0-dev libudev-dev
RUN cargo install probe-run cargo-flash

FROM $BASE_IMAGE AS runtime

ARG TARGET=thumbv7em-none-eabihf
ENV CARGO_HOME=/cargo_home
ENV DEFMT_LOG=debug

WORKDIR mikoto-bot

RUN apt update && apt install -y libusb-1.0-0-dev libudev-dev
RUN rustup target add $TARGET

COPY . .
COPY --from=prereqs /cargo_home /cargo_home

ENTRYPOINT ["cargo"]
