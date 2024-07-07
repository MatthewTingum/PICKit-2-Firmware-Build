FROM debian:12.5

# Keep this consistent with Docker tags
# Untagged changes should include the "-dirty" suffix
LABEL version="1.0"

RUN apt-get update  \
 && DEBIAN_FRONTEND=noninteractive \
 && apt-get install -y --no-install-recommends \
        ca-certificates \
        lib32stdc++6    \
        libc6-i386      \
        make            \
        wget            \
 && apt-get clean   \
 && rm -rf /var/lib/apt/lists/*

RUN wget -O /tmp/mplabc18-v3.40-linux-full-installer.run https://archive.org/download/mplabc18-v3.40-linux-full-installer/mplabc18-v3.40-linux-full-installer.run   \
 && chmod +x /tmp/mplabc18-v3.40-linux-full-installer.run   \
 && /tmp/mplabc18-v3.40-linux-full-installer.run --mode unattended --prefix /opt/microchip/mplabc18/v3.40   \
 && rm /tmp/mplabc18-v3.40-linux-full-installer.run

# Put c18 in the PATH
ENV PATH="/opt/microchip/mplabc18/v3.40/bin:/opt/microchip/mplabc18/v3.40/mpasm:$PATH"
