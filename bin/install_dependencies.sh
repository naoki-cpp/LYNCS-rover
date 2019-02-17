ARD_VER="1.8.7"
INSTALL_DIR="/usr/local/share/arduino"
echo "Downloading version $ARD_VER of the Arduino IDE..."
curl  http://downloads.arduino.cc/arduino-$ARD_VER-linux64.tar.xz | tar Jxfv -
git clone --depth 1 https://github.com/jrowberg/i2cdevlib.git
mv i2cdevlib/Arduino/* arduino-$ARD_VER/libraries
sudo mv arduino-$ARD_VER $INSTALL_DIR
sudo ln -s $INSTALL_DIR/arduino /usr/local/bin/arduino
