/*
 * Copyright 2025 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(tagged_iq_to_vector.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(c30ae02277949b4055c3fb8927fcc850)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/sic/tagged_iq_to_vector.h>
// pydoc.h is automatically generated in the build directory
#include <tagged_iq_to_vector_pydoc.h>

void bind_tagged_iq_to_vector(py::module& m)
{

    using tagged_iq_to_vector = ::gr::sic::tagged_iq_to_vector;


    py::class_<tagged_iq_to_vector,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<tagged_iq_to_vector>>(
        m, "tagged_iq_to_vector", D(tagged_iq_to_vector))

        .def(py::init(&tagged_iq_to_vector::make),
             py::arg("pre_offset"),
             py::arg("post_offset"),
             py::arg("max_gap"),
             D(tagged_iq_to_vector, make))


        ;
}
