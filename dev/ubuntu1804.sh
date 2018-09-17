#!/bin/sh

wget http://download.qt.io/official_releases/qt/5.11/5.11.1/qt-opensource-linux-x64-5.11.1.run
chmod +x qt-opensource-linux-x64-5.11.1.run
sudo ./qt-opensource-linux-x64-5.11.1.run

apt-get -y -qq install git cmake libzip-dev autoconf libtool libxml2 libxml2-dev libmatio4 libmatio-dev

apt-get -y -qq install python-dev
apt-get -y -qq install python3-dev

cd ~
wget https://netcologne.dl.sourceforge.net/project/boost/boost/1.63.0/boost_1_63_0.tar.gz
tar -xzvf boost_1_63_0.tar.gz
cd boost_1_63_0
./bootstrap.sh --with-libraries=date_time,filesystem,program_options,regex,system,test,thread
./b2
./b2 install

apt-get -y -qq install python-cheetah python-markdown

apt-get -y -qq install libfftw3-dev

cd ~
wget http://libvolk.org/releases/volk-1.3.tar.gz
tar -xzvf volk-1.3.tar.gz
cd volk-1.3
mkdir build && cd build
cmake ..
make
make install

cd ~
git clone https://github.com/analogdevicesinc/gnuradio
cd gnuradio
git checkout signal_source_phase
mkdir build && cd build
cmake -DENABLE_INTERNAL_VOLK:​BOOL=OFF -DENABLE_GR_FEC:BOOL=OFF -DENABLE_GR_DIGITAL:BOOL=OFF -DENABLE_GR_DTV:BOOL=OFF -DENABLE_GR_ATSC:BOOL=OFF -DENABLE_GR_AUDIO:BOOL=OFF -DENABLE_GR_CHANNELS:BOOL=OFF -DENABLE_GR_NOAA:BOOL=OFF -DENABLE_GR_PAGER:​BOOL=OFF -DENABLE_GR_TRELLIS:​BOOL=OFF -DENABLE_GR_VOCODER:​BOOL=OFF ..
make
make install

apt-get -y -qq install libffi-dev libmount-dev libpcre3-dev libglib2.0-dev libsigc++-2.0-dev libglibmm-2.4-dev doxygen

cd ~
git clone https://github.com/sigrokproject/libsigrok/
cd libsigrok
./autogen.sh
./configure --disable-all-drivers --enable-bindings --enable-cxx
make
make install

cd ~
wget http://sigrok.org/download/source/libsigrokdecode/libsigrokdecode-0.4.1.tar.gz
tar -xzvf libsigrokdecode-0.4.1.tar.gz
cd libsigrokdecode-0.4.1
./configure
make
make install

apt-get -y -qq install libglu1-mesa-dev

apt-get -y -qq install curl

#find the path to qmake
QMAKEPATH=`find /opt -path */bin/qmake`

cd ~
git clone https://github.com/osakared/qwt
cd qwt
git checkout qwt-6.1-multiaxes
git apply ../scopy/dev/qwt.diff
$QMAKEPATH qwt.pro
make
make install

cd ~
wget https://downloads.sourceforge.net/project/qwtpolar/qwtpolar/1.1.1/qwtpolar-1.1.1.tar.bz2
tar xvjf qwtpolar-1.1.1.tar.bz2
cd qwtpolar-1.1.1
curl -o qwtpolar-qwt-6.1-compat.patch https://raw.githubusercontent.com/analogdevicesinc/scopy-flatpak/master/qwtpolar-qwt-6.1-compat.patch
patch -p1 < qwtpolar-qwt-6.1-compat.patch
patch -p1 < ../scopy/dev/qwtpolar.diff qwtpolarconfig.pri
gedit qwtpolarconfig.pri
$QMAKEPATH qwtpolar.pro
make
make install

apt-get -y -qq install libusb-1.0-0-dev
cd ~
git clone https://github.com/analogdevicesinc/libiio
cd libiio && mkdir build && cd build
cmake -DCMAKE_INSTALL_LIBDIR:STRING=lib -DINSTALL_UDEV_RULE:BOOL=OFF -DWITH_TESTS:BOOL=OFF -DWITH_DOC:BOOL=OFF -DWITH_IIOD:BOOL=OFF -DWITH_LOCAL_BACKEND:BOOL=OFF -DWITH_MATLAB_BINDINGS_API:BOOL=OFF ..
make
make install

apt-get -y -qq install flex

apt-get -y -qq install bison

cd ~
git clone https://github.com/analogdevicesinc/libad9361-iio
cd libad9361-iio
mkdir build && cd build
cmake ..
make
make install

cd ~
git clone https://github.com/analogdevicesinc/gr-iio
cd gr-iio
mkdir build && cd build
cmake ..
make
make install

QTPATHS=`find /opt -path */bin/qtpaths`
PREFIXPATH=`$QTPATHS --install-prefix`

cd ~
git clone https://github.com/analogdevicesinc/scopy.git
cd scopy
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH:STRING=$PREFIXPATH ..
make
./scopy


