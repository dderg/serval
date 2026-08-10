FROM python:3.12-slim-bookworm

ARG OMP_VERSION=17.2.11
ENV BUN_INSTALL=/opt/bun \
    PATH=/opt/bun/bin:/opt/bun/install/global/node_modules/.bin:/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl git login passwd tini unzip \
    && rm -rf /var/lib/apt/lists/* \
    && curl -fsSL https://bun.sh/install | bash \
    && bun add --global "@oh-my-pi/pi-coding-agent@${OMP_VERSION}"

WORKDIR /app
COPY pyproject.toml ./
COPY src ./src
RUN pip install --no-cache-dir .

COPY entrypoint.sh /usr/local/bin/serval-entrypoint
RUN chmod +x /usr/local/bin/serval-entrypoint

VOLUME ["/data"]
EXPOSE 8080 8081
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/serval-entrypoint"]
CMD ["serval-bot"]
