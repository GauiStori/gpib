## Important

The user space linux-gpib code is now a Debian package
https://tracker.debian.org/pkg/linux-gpib-user

and the kernel modules are now in the official kernel tree.

The modules are however not compiled. 

install the linux-source-6.16 debian package.

cp /usr/src/linux-source-6.16.tar.xz ~/
tar xf linux-source-6.16.tar.xz
cd linux-source-6.16
make clean
make mrproper
cp /boot/config-6.16.12+deb14+1-amd64 .config
make oldconfig
make menuconfig
Go to Device drivers/Staging drivers and select Linux GPIB drivers as modules
Build the kernel with:

nice make -j`nproc` bindeb-pkg

See: https://wiki.debian.org/BuildADebianKernelPackage
It says it is outdated but it works.

## Welcome to the linux-gpib sourceforge git repository.

This replaces the SVN repository which is no longer being updated.
The last commit to SVN is
[r2106] = git commit  b4cbd1387e7e... Add gpio pin offset parameter for RPi5 (Marcello Carla')

### To clone the git repo:


```
git clone git://git.code.sf.net/p/linux-gpib/git linux-gpib-git
```

### To pull the latest commits:

```
git pull
```

### To list the commits made since a release tag (e.g. v4_3_6):

```
git log HEAD ^v4_3_6
```

### To build the user part:

```
cd linux-gpib-git/linux-gpib-user
./bootstrap
./configure --sysconfdir=/etc
make
sudo make install
```

For more details and options see linux-gpib-git/linux-gpib-user/INSTALL

### To build the kernel part:

```
cd linux-gpib-git/linux-gpib-kernel
make
sudo make install
```

For more details and options see linux-gpib-git/linux-gpib-kernel/INSTALL
