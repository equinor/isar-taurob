FROM ghcr.io/astral-sh/uv:0.13.7@sha256:96f40d3904aad8d0bd010539001d36c8e2188b981e1334471a90c4a1d93bb490 AS uv
FROM python:3.15-slim@sha256:65a65fc7b30b06cb6df4930fd4650697cb23f4c2698ab81fc87e4b031bc36928 AS build

COPY --from=uv /uv /uvx /bin/

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends git build-essential gcc \
    pkg-config libcairo2-dev libgirepository-2.0-dev \
    && rm -rf /var/lib/apt/lists/*

COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --extra recording --no-dev --no-install-project

COPY . .
RUN --mount=source=./.git,target=./.git,type=bind uv sync --frozen --extra recording --no-editable --no-dev

FROM python:3.15-slim@sha256:65a65fc7b30b06cb6df4930fd4650697cb23f4c2698ab81fc87e4b031bc36928
WORKDIR /app
COPY --from=build /app/.venv /app/.venv
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg libcairo2 libgirepository-2.0-0 gir1.2-gst-plugins-bad-1.0 \
    gstreamer1.0-nice gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    && rm -rf /var/lib/apt/lists/*
ENV PATH="/app/.venv/bin:$PATH"

EXPOSE 3000

# Env variable for ISAR to know it is running in docker
ENV IS_DOCKER=true

# Add non-root user
RUN useradd --create-home --uid 1000 --shell /bin/bash app
RUN chown -R 1000 /app
RUN chmod 755 /app
USER 1000

CMD ["isar-start"]
