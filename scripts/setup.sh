#
# Copyright (C) 2011 Georgia Institute of Technology, University of Utah, Weill Cornell Medical College
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

#!/bin/bash

# Check for compilation dependencies
echo "Checking for dependencies..."

sudo apt-get update -q=2
sudo apt-get install automake libtool autoconf autotools-dev build-essential qt4-dev-tools libboost-dev libboost-program-options-dev libgsl0-dev bison flex libncurses5-dev libqwt5-qt4-dev libqt4-gui libqt4-core libqt4-dev

if [ $? -eq 0 ]; then
	echo "----->Dependencies installed."
else
	echo "----->Dependency installation failed."
	exit
fi

sudo ln -s /usr/lib/libqwt-qt4.so.5 /usr/lib/libqwt.so

# Installing HDF5
echo "----->Checking for HDF5"

if [ -f "/usr/include/hdf5.h" ]; then
	echo "----->HDF5 already installed."
else
	echo "----->Installing HDF5..."
	cd hdf
	tar xf hdf5-1.8.4.tar.bz2
	cd hdf5-1.8.4
	./configure --prefix=/usr
	sudo make
	sudo make install
	cd ../../
	if [ $? -eq 0 ]; then
			echo "----->HDF5 installed."
	else
		echo "----->HDF5 installation failed."
		exit
	fi
fi

# Start configuring - by default configured to run on non-RT kernel
echo "----->Starting RTXI installation..."
./autogen.sh

echo "----->Kernel configuration..."
echo "1. RTAI+Comedi"
echo "2. RTAI+Analogy"
echo "3. Xenomai+Analogy"
echo "4. POSIX (Non-RT)+Analogy"
echo "----->Please select your configuration:"
read -n 1 kernel

if [ $kernel -eq "1" ]; then
	./configure --enable-comedi --enable-rtai
elif [ $kernel -eq "2" ]; then
	./configure --enable-rtai --enable-analogy
elif [ $kernel -eq "3" ]; then
	./configure --enable-xenomai --enable-analogy
elif [ $kernel -eq "4" ]; then
	./configure --disable-xenomai --enable-posix --enable-analogy --disable-comedi
else
	echo "Invalid configuration."
	exit
fi

# Compile RTXI
sudo make -C ./

if [ $? -eq 0 ]; then
	echo "----->RTXI intallation successful."
else
	echo "----->RTXI installation failed."
	exit
fi

# Install RTXI
sudo make install -C ./

if [ $? -eq 0 ]; then
	echo "----->RTXI intallation successful."
else
	echo "----->RTXI installation failed."
	exit
fi

echo "----->Putting things into place."
sudo cp libtool /usr/local/lib/rtxi/
sudo cp rtxi.conf /etc/
sudo cp /usr/xenomai/sbin/analogy_config /usr/sbin/
sudo cp ./scripts/rtxi_load_analogy /etc/init.d/
sudo update-rc.d rtxi_load_analogy defaults

echo "----->Loading device."
./scripts/rtxi_load_analogy

if [ $? -eq 0 ]; then
	echo "----->RTXI intallation successful."
else
	echo "----->RTXI installation failed."
	exit
fi

echo "----->Happy Sciencing!"