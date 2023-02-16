from ubuntu:18.04

# !!! WARNING !!!
# This is modified for OSTAR use. Please use the offical scenario runner if you dont know what is going on here.

ARG DEBIAN_FRONTEND=noninteractive
ARG GRPC_TRACE=all
ARG GRPC_VERBOSITY=DEBUG

# Install base libs
run apt-get update && apt-get install --no-install-recommends -y libpng16-16=1.6.34-1ubuntu0.18.04.2 \
libtiff5=4.0.9-5ubuntu0.9 libjpeg8=8c-2ubuntu8 build-essential=12.4ubuntu1 wget=1.19.4-1ubuntu2.2 git=1:2.17.1-1ubuntu0.16 \
 python3.6 python3.6-dev python3-pip libxerces-c-dev \
 && rm -rf /var/lib/apt/lists/* 

# Install python requirements
run pip3 install --user setuptools==46.3.0 wheel==0.34.2 && pip3 install py_trees==0.8.3 networkx==2.2 pygame==1.9.6 \
    six numpy psutil shapely==1.7.1 xmlschema==1.0.18 ephem tabulate==0.8.7 simple-watchdog-timer\
    grpcio grpcio-tools CherryPy \
    && mkdir -p /app/scenario_runner 

# Install scenario_runner 
copy . /app/scenario_runner


# setup environment :
# 
#   CARLA_HOST :    uri for carla package without trailing slash. 
#                   For example, "https://carla-releases.s3.eu-west-3.amazonaws.com/Linux".
#                   If this environment is not passed to docker build, the value
#                   is taken from CARLA_VER file inside the repository.
#
#   CARLA_RELEASE : Name of the package to be used. For example, "CARLA_0.9.9".
#                   If this environment is not passed to docker build, the value
#                   is taken from CARLA_VER file inside the repository.
# 
#
#  It's expected that $(CARLA_HOST)/$(CARLA_RELEASE).tar.gz is a downloadable resource.
#

# Removed because local firewall prevented download, if amazonaws is not blocked remove the # and use original script

#env CARLA_HOST ""
#env CARLA_RELEASE ""

# Extract and install python API and resources from CARLA
#run export DEFAULT_CARLA_HOST="$(sed -e 's/^\s*HOST\s*=\s*//;t;d' /app/scenario_runner/CARLA_VER)" && \
#    echo "$DEFAULT_CARLA_HOST" && \
#    export CARLA_HOST="${CARLA_HOST:-$DEFAULT_CARLA_HOST}" && \
#    export DEFAULT_CARLA_RELEASE="$(sed -e 's/^\s*RELEASE\s*=\s*//;t;d' /app/scenario_runner/CARLA_VER)" && \
#    export CARLA_RELEASE="${CARLA_RELEASE:-$DEFAULT_CARLA_RELEASE}" && \
#    echo "$CARLA_HOST/$CARLA_RELEASE.tar.gz" && \
#    wget -qO- "$CARLA_HOST/$CARLA_RELEASE.tar.gz" | tar -xzv PythonAPI/carla -C / && \
#    mv /PythonAPI/carla /app/ && \
#    python3 -m easy_install --no-find-links --no-deps "$(find /app/carla/ -iname '*py3.*.egg' )"

# manual do what the commented run command would do
# look into the CARLA_VER file and manual download the tar
# untar it and copy into this file (look at the above line with wget and the next on)
RUN mv /app/scenario_runner/PythonAPI/carla /app/ && \
    python3 -m easy_install --no-find-links --no-deps "$(find /app/carla/ -iname '*py3.*.egg' )"

# Setup working environment
workdir /app/scenario_runner
run mkdir record
env PYTHONPATH "${PYTHONPATH}:/app/carla/agents:/app/carla"
env PYTHONIOENCODING=utf8
entrypoint ["script/startup.sh"]
# Default url for the carla instance, modify or overwrite for your setup
cmd ["ts-gpu-bs.intra.dlr.de"]

