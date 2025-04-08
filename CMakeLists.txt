cmake_minimum_required(VERSION 3.10)
project(LQR_Controller C)

set(SOURCES
    helpers.c
    lqr_solver.c
)

add_executable(lqr_solver ${SOURCES})