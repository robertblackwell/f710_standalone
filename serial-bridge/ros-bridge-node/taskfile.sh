function build {
	cd ../..
	# colcon build --packages-select cpp_serial_bridge --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	colcon build --packages-select cpp_serial_bridge --symlink-install \
		--event-handlers console_direct+ \
		--cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON \
		--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	source install/setup.bash
}
function build_debug {
	cd ../..
	colcon build --packages-select cpp_pubsub --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
	source install/setup.bash
}

function debug {
	ros2 run --prefix 'gdbserver localhost:3000' cpp_pubsub $1
}

function run {
	ros2 run  cpp_pubsub $1
}

"$@"