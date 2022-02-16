1.3rdparty编译问题
虚拟机：VMware 12.5.7
Ubuntu：16.04 LTS

在Windows的共享目录中，编译程序无法进行符号链接，会出现类似错误：
Symlink_library: system error: operation not supported

主要是flann，pcl，opencv
Boost不会遇到这样的问题。

解决方法：把相关的库从Windows共享目录中拷贝到系统目录中，然后编译安装到usr目录下。

1.Boost编译 1.63.0
./bootstrap.sh --with-libraries=all --with-toolset=gcc
./b2 toolset=gcc
./b2 install --prefix=/usr
--prefix=/usr用来指定boost的安装目录，不加此参数的话默认的头文件在/usr/local/include/boost目录下，库文件在/usr/local/lib/目录下。这里把安装目录指定为--prefix=/usr则boost会直接安装到系统头文件目录和库文件目录下，可以省略配置环境变量。


2.Eigen   3.3.1
$ mkdir build 
$ cd build 
$ cmake ../ 
$ sudo make install


3.Opencv   2.4.13
1.$sudo apt-get install build-essential  
2.$sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev  
3.$sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev  

1.$cd ~/opencv-3.1.0  
2.$mkdir release  
3.$cd release  
4.$cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr cmake -DENABLE_PRECOMPILED_HEADERS=OFF..  
5.$make  
6.$sudo make install  


2017-08-29  upgrade opencv2 to opencv3 for multiple object tracking apps.



4.PCL  1.8.0
cd ~/Documents/pcl 
mkdir release 
cd release 
cmake -D CMAKE_BUILD_TYPE=RELEASE -DBUILD_GPU=ON -DBUILD_apps=ON -D CMAKE_INSTALL_PREFIX=/usr  .. 
make 
sudo make install 


5.CGAL    4.10
sudo apt-get install libcgal-dev         #install the CGAL library
sudo apt-get install libcgal-demo       #install the CGAL demos

