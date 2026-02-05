# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bee_osif)

target_sources(app PRIVATE
    src/osif_mem.c
    src/osif_msgq.c
    src/osif_mutex.c
    src/osif_sched.c
    src/osif_semaphore.c
    src/osif_task_apis.c
    src/osif_task_signal.c
    src/osif_timer.c
)
