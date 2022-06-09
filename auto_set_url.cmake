macro(auto_set_url TARGET)
   file(RELATIVE_PATH URL_REL_PATH  "${PICO_PATH}" "${CMAKE_CURRENT_LIST_DIR}")
   pico_set_program_url(${TARGET} "${PICO_URL_BASE}/${URL_REL_PATH}")
endmacro()
