xcode project file was made by running the following from within the xcode directory:

export OPENSSL_ROOT_DIR=/usr/local/Cellar/openssl/1.0.2l/
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig
cmake -G "Xcode" ../..

in xcode, select ALL_BUILD > Edit Scheme... >  Run > Info > Executable > pid