1.3rdparty��������
�������VMware 12.5.7
Ubuntu��16.04 LTS

��Windows�Ĺ���Ŀ¼�У���������޷����з������ӣ���������ƴ���
Symlink_library: system error: operation not supported

��Ҫ��flann��pcl��opencv
Boost�����������������⡣

�������������صĿ��Windows����Ŀ¼�п�����ϵͳĿ¼�У�Ȼ����밲װ��usrĿ¼�¡�

1.Boost���� 1.63.0
./bootstrap.sh --with-libraries=all --with-toolset=gcc
./b2 toolset=gcc
./b2 install --prefix=/usr
--prefix=/usr����ָ��boost�İ�װĿ¼�����Ӵ˲����Ļ�Ĭ�ϵ�ͷ�ļ���/usr/local/include/boostĿ¼�£����ļ���/usr/local/lib/Ŀ¼�¡�����Ѱ�װĿ¼ָ��Ϊ--prefix=/usr��boost��ֱ�Ӱ�װ��ϵͳͷ�ļ�Ŀ¼�Ϳ��ļ�Ŀ¼�£�����ʡ�����û���������


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

