#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import shutil


def execute_build(args):
    """
    Create docker build command based on user input arguments
    and execute the command

    Args:
        args (argparse.Namespace): Build arguments
    """
    if not os.path.exists(args.docker_file):
        print('Dockerfile %s not found! Exiting' % args.docker_file)
        return

    # copy requirements file from parent into docker folder
    cwd = os.getcwd()
    #shutil.copy(cwd + '/../requirements.txt', cwd)

    cmd = 'DOCKER_BUILDKIT=1 docker build '
    #cmd += '--ssh default '
    cmd += '--network=host '
    if args.no_cache:
        cmd += '--no-cache '
    cmd += '--build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" '
    cmd += '-t %s -f %s .' % (args.image, args.docker_file)



    print('command = \n\n', cmd)

    if not args.dry_run:
        os.system(cmd)



if __name__ == '__main__':
    #default_image_name = "jumping_from_pixels:latest"
    default_image_name = "gmargo11/jumping_from_pixels:public"
    #default_image_name = "gmargo11/jumping_from_pixels:release"

    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--image', type=str,
                        default=default_image_name,
                        help='name for new docker image')

    parser.add_argument('--no_cache', action='store_true',
                        help='0 if should build without using cache')

    parser.add_argument('-f', '--docker_file', type=str,
                        #default='pycheetah_dev.dockerfile',
                        default='pycheetah_public.dockerfile',
                        #default='pycheetah.dockerfile',
                        help='which Dockerfile to build from')

    parser.add_argument('-d', '--dry_run', action='store_true',
                        help='1 if we should only print the build command '
                             'without executing')

    parser.add_argument('--ssh',  type=str, default='default')

    args = parser.parse_args()
    execute_build(args)
