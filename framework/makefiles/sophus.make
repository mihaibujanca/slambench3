Sophus_INCLUDE_DIR=${DEPS_DIR}/Sophus/include
Sophus_INCLUDE_DIRS=${DEPS_DIR}/Sophus/include
Sophus_DIR=${DEPS_DIR}/Sophus/share/sophus/cmake

${REPOS_DIR}/Sophus :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/Sophus -rf
	git clone "https://github.com/strasdat/Sophus.git" ${REPOS_DIR}/Sophus
	cd ${REPOS_DIR}/Sophus && git checkout b474f05f839c0f63c281aa4e7ece03145729a2cd
	sed -i.bak "s/-Wall//" ${REPOS_DIR}/Sophus/CMakeLists.txt


${DEPS_DIR}/Sophus : ${REPOS_DIR}/Sophus
	if [ ! -e $(EIGEN3_INCLUDE_DIR) ] ; \
	then \
		echo "ERROR: Sophus requires EIGEN to be fully functional (make eigen).";\
		exit 1;\
	fi;
	cd ${REPOS_DIR}/Sophus && mkdir build -p && rm build/* -rf
	cd ${REPOS_DIR}/Sophus/build && cmake .. "-DCMAKE_INSTALL_PREFIX=$@" -DCMAKE_BUILD_TYPE=Release  -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}  -DCMAKE_CXX_FLAGS="-Wno-error=int-in-bool-context"
	cd ${REPOS_DIR}/Sophus/build && make -j2
	mkdir -p $@
	cd ${REPOS_DIR}/Sophus/build && make install

Sophus :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
.PHONY: sophus
