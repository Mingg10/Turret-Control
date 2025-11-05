# mjbots_tests

setup SFTP plugin to copy code to the raspberry pi. 

SSH to the pi.

``` bash
cd mjbots_tests

mkdir build

cd build

cmake ..
```

This will generate the makefiles for compilation. You can then compile the code to run spool test with the commands below.

``` bash

make spool_test.
```

Make sure to recompile the code when you push code changes to the pi.