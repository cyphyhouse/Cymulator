# This is an auto generated Dockerfile for gazebo:gzweb9
# generated from docker_images/create_gzweb_image.Dockerfile.em
FROM gazebo:libgazebo9-xenial

# update source list for nodejs
RUN apt-get update && apt-get install -q -y --no-install-recommends \
        apt-transport-https \
        ca-certificates \
        curl \
    && apt-get clean
RUN curl -sL https://deb.nodesource.com/setup_10.x | bash -

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libjansson-dev \
    libtinyxml-dev \
    nodejs \
    pkg-config \
    psmisc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# install gazebo packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libgazebo9-dev=9.13.0-1* \
    && rm -rf /var/lib/apt/lists/*

# clone gzweb
ENV GZWEB_WS /root/gzweb
RUN git clone https://github.com/osrf/gzweb.git --branch gzweb_1.4.0 $GZWEB_WS
WORKDIR $GZWEB_WS

RUN . /usr/share/gazebo/setup.sh
# build gzweb
RUN xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m local

# setup environment
EXPOSE 8080
EXPOSE 7681

# run gzserver and gzweb
CMD gzserver --verbose & npm start
