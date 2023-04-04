include_directories(include)
add_library(MITAMD SHARED
        # QpProblem.cpp
        # ProblemGenerator.cpp
        # eigenvalues.cpp
        # CholeskyDenseSolver.cpp
        # CholeskySparseSolver.cpp
        # SparseMatrixMath.cpp
        #OsqpTest.cpp
        #qpOASES.cpp
        #ThreadGroup.cpp
        src/amd_1.c
        src/amd_2.c
        src/amd_aat.c
        src/amd_control.c
        src/amd_defaults.c
        src/amd_info.c
        src/amd_order.c
        src/amd_post_tree.c
        src/amd_postorder.c
        src/amd_preprocess.c
        src/amd_valid.c
        src/SuiteSparse_config.c)


#target_link_libraries(JCQP OsqpEigen::OsqpEigen osqp::osqp qpOASES pthread)
target_link_libraries(MITAMD pthread)