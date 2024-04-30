# Do not add --platform=linux/blabla since this is intended for multiplatform builds
FROM tomnotch/bipvrobotics-base-image:ros1
ENV HOME_FOLDER=/root
WORKDIR $HOME_FOLDER/

# Fix apt install stuck problem
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy the home folder config files
COPY --from=home-folder-config . $HOME_FOLDER/

# Install pre-commit hooks
RUN git init && \
    pre-commit install-hooks && \
    rm -rf .git .pre-commit-config.yaml

# update all obsolete packages to latest, install sudo, and cleanup
RUN apt-get update -o Acquire::Check-Valid-Until=false -o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true && \
    apt-get full-upgrade -y && \
    apt-get install -y sudo ca-certificates && \
    apt-get autoremove -y

# Install libdw and git large file system
RUN apt-get install -y libdw-dev git-lfs

# Install Ceres Solver from source
RUN apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev && \
    git clone --recursive https://ceres-solver.googlesource.com/ceres-solver ${HOME_FOLDER}/ceres-solver -b 2.2.0 && \
    mkdir -p ${HOME_FOLDER}/ceres-solver/build && \
    cd ${HOME_FOLDER}/ceres-solver/build && \
    cmake .. && \
    make install -j$(($(nproc)-1)) && \
    rm -rf ${HOME_FOLDER}/ceres-solver

# Install SymEngine from source
RUN apt-get install -y cmake libgmp-dev && \
    git clone --recursive https://github.com/symengine/symengine.git ${HOME_FOLDER}/symengine && \
    mkdir -p ${HOME_FOLDER}/symengine/build && \
    cd ${HOME_FOLDER}/symengine/build && \
    cmake -DBUILD_TESTS:BOOL=OFF \
          -DBUILD_BENCHMARKS:BOOL=OFF \
          -DBUILD_BENCHMARKS_GOOGLE:BOOL=OFF \
          -DBUILD_SHARED_LIBS:BOOL=ON \
          .. && \
    make install -j$(($(nproc)-1)) && \
    rm -rf ${HOME_FOLDER}/symengine

# # Install ArUco from source
# RUN wget https://sourceforge.net/projects/aruco/files/latest -O ${HOME_FOLDER}/aruco.zip && \
#     unzip ${HOME_FOLDER}/aruco.zip && \
#     mv ${HOME_FOLDER}/aruco-* ${HOME_FOLDER}/aruco && \
#     mkdir -p ${HOME_FOLDER}/aruco/build && \
#     cd ${HOME_FOLDER}/aruco/build && \
#     cmake .. && \
#     make install -j$(($(nproc)-1)) && \
#     rm -rf ${HOME_FOLDER}/aruco.zip ${HOME_FOLDER}/aruco

ENV CODE_FOLDER=elec-5660-project-3-phase-1
RUN echo "source ${HOME_FOLDER}/${CODE_FOLDER}/devel/setup.zsh" >> ${HOME_FOLDER}/.zshrc && \
    echo "source ${HOME_FOLDER}/${CODE_FOLDER}/devel/setup.bash" >> ${HOME_FOLDER}/.bashrc

# Set the default shell to zsh
SHELL [ "/bin/zsh", "-c" ]

# # move fixuid config
# RUN mv $HOME_FOLDER/fixuid-config.yml /etc/fixuid/config.yml

# Entrypoint command
# ENTRYPOINT [ "/bin/sh" , "-c", "fixuid; /bin/zsh" ]
ENTRYPOINT [ "/bin/zsh" ]
