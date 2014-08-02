#!/bin/bash

qmake-qt4 qt_gl.pro -r -spec linux-g++

make

DIR=$(vagrant ssh-config | grep IdentityFile  | awk '{print $2}')

ssh -X -i "${DIR}"/.vagrant.d/insecure_private_key -l vagrant -p 2222 127.0.0.1 -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no "cd /vagrant/qt_gl/ && export LIBGL_DEBUG=verbose; && ./qt_gl"
