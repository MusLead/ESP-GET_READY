# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources} INCLUDE_DIRS "." 
                        EMBED_TXTFILES certs/isrg-root-x1.pem certs/ca.crt certs/server.key certs/server.crt certs/client.key certs/client.crt 
                        PRIV_REQUIRES esp_wifi esp_http_server)
