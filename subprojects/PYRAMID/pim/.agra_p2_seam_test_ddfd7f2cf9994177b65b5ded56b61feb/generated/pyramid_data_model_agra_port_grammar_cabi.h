#ifndef PYRAMID_DATA_MODEL_AGRA_PORT_GRAMMAR_CABI_H
#define PYRAMID_DATA_MODEL_AGRA_PORT_GRAMMAR_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_data_model_agra_port_grammar_Identifier_c {
  pyramid_str_t id;
} pyramid_data_model_agra_port_grammar_Identifier_c;

typedef struct pyramid_data_model_agra_port_grammar_Ack_c {
  uint8_t success;
} pyramid_data_model_agra_port_grammar_Ack_c;

typedef struct pyramid_data_model_agra_port_grammar_Query_c {
} pyramid_data_model_agra_port_grammar_Query_c;

void pyramid_data_model_agra_port_grammar_Identifier_c_free(pyramid_data_model_agra_port_grammar_Identifier_c* value);
void pyramid_data_model_agra_port_grammar_Ack_c_free(pyramid_data_model_agra_port_grammar_Ack_c* value);
void pyramid_data_model_agra_port_grammar_Query_c_free(pyramid_data_model_agra_port_grammar_Query_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_AGRA_PORT_GRAMMAR_CABI_H */
