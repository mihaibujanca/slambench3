FROM archlinux:latest

RUN pacman -Syy
RUN pacman -Syu --noconfirm
RUN pacman -S   --noconfirm linux-lts nvidia-lts
RUN pacman -S   --noconfirm base-devel cmake sudo
RUN pacman -S   --noconfirm opencv boost yaml-cpp
RUN pacman -S   --noconfirm eigen wget git vim python2 unzip python-numpy python2-numpy qt vtk glew hdf5 google-glog netcdf glut
RUN pacman -S   --noconfirm gflags protobuf gcc8
RUN pacman -S   --noconfirm cuda mesa-demos
RUN git config --global user.name "John Doe"
RUN git config --global user.email "john.doe@email.com"

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

RUN pacman -S   --noconfirm python doxygen graphviz freeglut glu java-environment
RUN useradd -ms /bin/bash user
RUN mkdir /tmp/openni2 && chown user /tmp/openni2
USER user
RUN cd /tmp/openni2 && git clone https://aur.archlinux.org/openni2.git && cd openni2 && makepkg -c && ls
USER root
RUN cd /tmp/openni2/openni2 && pacman -U --noconfirm openni2-2.2.0-1-x86_64.pkg.tar.zst
RUN pwd

RUN pacman -S   --noconfirm gtk2 pkg-config proj openmpi gdal pdal unixodbc pugixml
Run pacman -S   --noconfirm pkg-config pacutils
RUN pacinstall --no-confirm --resolve-conflicts=all openblas

RUN git clone https://github.com/Paul92/slambench3.git && cd slambench3 && git checkout -b semanticfusion_clean2 origin/semanticfusion_clean2
RUN rm /usr/sbin/pkg-config
RUN cd slambench3 && make deps
RUN rm -rf slambench3/benchmarks/semanticfusion
RUN cd slambench3 && echo y | make usecases semanticfusion
RUN cd slambench3 && make slambench -j8
RUN cd slambench3 && make slambench APPS="semanticfusion" -j8
