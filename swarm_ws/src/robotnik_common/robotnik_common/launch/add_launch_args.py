# Copyright (c) 2023, Robotnik Automation S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from dataclasses import dataclass, field


@dataclass
class ExtendedArgument:
    """ Extended Arguments item dataclass """
    name: str
    default_value: str
    description: str
    # Use environment variable
    use_env: bool = field(default=True)
    # Environment variable name to use
    environment: str = field(default=None)

    def __post_init__(self):
        if not self.environment:
            self.environment = self.name.upper()


class AddArgumentParser():
    """ Add the argument with environment variable """

    def __init__(
        self,
        ld: LaunchDescription,
    ):
        self._arg_list = []
        self._ld = ld

    def add_arg(
        self,
        arg: ExtendedArgument,
    ):
        self._arg_list.append(arg)

    def __add_launch_arg(
        self,
        arg: ExtendedArgument,
    ):
        if not arg.environment:
            arg.environment = arg.name.upper()
        if arg.use_env:
            default_value = EnvironmentVariable(
                name=arg.environment,
                default_value=arg.default_value,
            )
            arg = DeclareLaunchArgument(
                name=arg.name,
                description=arg.description,
                default_value=default_value,
            )
        else:
            arg = DeclareLaunchArgument(
                name=arg.name,
                description=arg.description,
                default_value=arg.default_value,
            )
        self._ld.add_action(arg)
        pass

    def process_arg(
        self,
    ):
        params = {}
        for arg in self._arg_list:
            self.__add_launch_arg(arg)
            params[arg.name] = LaunchConfiguration(arg.name)
        return params


def add_launch_args(
    ld: LaunchDescription,
    params: list[
        tuple[
            str,
            str,
            str
        ]
    ]
):
    # name, description, default_value

    ret = {}

    for param in params:
        # Declare the launch options
        ld.add_action(
            DeclareLaunchArgument(
                name=param[0],
                description=param[1],
                default_value=EnvironmentVariable(
                    param[0].upper(),
                    default_value=param[2],
                ),
            )
        )

        # Get the launch configuration variables
        ret[param[0]] = LaunchConfiguration(param[0])

    return ret
