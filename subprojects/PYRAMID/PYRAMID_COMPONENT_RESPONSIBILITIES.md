# PYRAMID Technical Standard Component Responsibilities

- Source: `ref/20250224-PYRAMID_Technical_Standard_V1-O.md`
- Scope: Components in `5.4.2.x` with responsibilities in `5.4.2.x.4`
- Responsibility ID format: `PYR-RESP-####` (globally unique)

Total components: 73
Total responsibilities: 885

## Navigation

- [COMP-001 - Anomaly Detection](#comp-001---anomaly-detection-5421)
- [COMP-002 - Asset Transitions](#comp-002---asset-transitions-5422)
- [COMP-003 - Authorisation](#comp-003---authorisation-5423)
- [COMP-004 - Collision Avoidance](#comp-004---collision-avoidance-5424)
- [COMP-005 - Collision Prediction](#comp-005---collision-prediction-5425)
- [COMP-006 - Communication Links](#comp-006---communication-links-5426)
- [COMP-007 - Communicator](#comp-007---communicator-5427)
- [COMP-008 - Conflict Resolution](#comp-008---conflict-resolution-5428)
- [COMP-009 - Countermeasures](#comp-009---countermeasures-5429)
- [COMP-010 - Cryptographic Materials](#comp-010---cryptographic-materials-54210)
- [COMP-011 - Cryptographic Methods](#comp-011---cryptographic-methods-54211)
- [COMP-012 - Cyber Defence](#comp-012---cyber-defence-54212)
- [COMP-013 - Data Distribution](#comp-013---data-distribution-54213)
- [COMP-014 - Data Fusion](#comp-014---data-fusion-54214)
- [COMP-015 - Destructive Effects](#comp-015---destructive-effects-54215)
- [COMP-016 - Effectors](#comp-016---effectors-54216)
- [COMP-017 - Environment Infrastructure](#comp-017---environment-infrastructure-54217)
- [COMP-018 - Environment Integration](#comp-018---environment-integration-54218)
- [COMP-019 - Environmental Conditioning](#comp-019---environmental-conditioning-54219)
- [COMP-020 - Flights](#comp-020---flights-54220)
- [COMP-021 - Fluids](#comp-021---fluids-54221)
- [COMP-022 - Formations](#comp-022---formations-54222)
- [COMP-023 - Geography](#comp-023---geography-54223)
- [COMP-024 - Health Assessment](#comp-024---health-assessment-54224)
- [COMP-025 - HMI Dialogue](#comp-025---hmi-dialogue-54225)
- [COMP-026 - Human Interaction](#comp-026---human-interaction-54226)
- [COMP-027 - Information Brokerage](#comp-027---information-brokerage-54227)
- [COMP-028 - Information Presentation](#comp-028---information-presentation-54228)
- [COMP-029 - Interlocks](#comp-029---interlocks-54229)
- [COMP-030 - Inventory](#comp-030---inventory-54230)
- [COMP-031 - Jettison](#comp-031---jettison-54231)
- [COMP-032 - Lights](#comp-032---lights-54232)
- [COMP-033 - Location and Orientation](#comp-033---location-and-orientation-54233)
- [COMP-034 - Mass and Balance](#comp-034---mass-and-balance-54234)
- [COMP-035 - Mechanical Positioning](#comp-035---mechanical-positioning-54235)
- [COMP-036 - Navigation Sensing](#comp-036---navigation-sensing-54236)
- [COMP-037 - Network Routes](#comp-037---network-routes-54237)
- [COMP-038 - Networks](#comp-038---networks-54238)
- [COMP-039 - Objectives](#comp-039---objectives-54239)
- [COMP-040 - Observability](#comp-040---observability-54240)
- [COMP-041 - Operational Rules and Limits](#comp-041---operational-rules-and-limits-54241)
- [COMP-042 - Pointing](#comp-042---pointing-54242)
- [COMP-043 - Power](#comp-043---power-54243)
- [COMP-044 - Propulsion](#comp-044---propulsion-54244)
- [COMP-045 - Reference Times](#comp-045---reference-times-54245)
- [COMP-046 - Release Aiming](#comp-046---release-aiming-54246)
- [COMP-047 - Release Effecting](#comp-047---release-effecting-54247)
- [COMP-048 - Routes](#comp-048---routes-54248)
- [COMP-049 - Semantic Translation](#comp-049---semantic-translation-54249)
- [COMP-050 - Sensing](#comp-050---sensing-54250)
- [COMP-051 - Sensor Data Interpretation](#comp-051---sensor-data-interpretation-54251)
- [COMP-052 - Sensor Products](#comp-052---sensor-products-54252)
- [COMP-053 - Sensors](#comp-053---sensors-54253)
- [COMP-054 - Signature](#comp-054---signature-54254)
- [COMP-055 - Spatial Correction](#comp-055---spatial-correction-54255)
- [COMP-056 - Spectrum](#comp-056---spectrum-54256)
- [COMP-057 - Storage](#comp-057---storage-54257)
- [COMP-058 - Stores Release](#comp-058---stores-release-54258)
- [COMP-059 - Susceptibility](#comp-059---susceptibility-54259)
- [COMP-060 - Tactical Objects](#comp-060---tactical-objects-54260)
- [COMP-061 - Target Engagement](#comp-061---target-engagement-54261)
- [COMP-062 - Tasks](#comp-062---tasks-54262)
- [COMP-063 - Test](#comp-063---test-54263)
- [COMP-064 - Threats](#comp-064---threats-54264)
- [COMP-065 - Trajectory Prediction](#comp-065---trajectory-prediction-54265)
- [COMP-066 - Undercarriage](#comp-066---undercarriage-54266)
- [COMP-067 - User Accounts](#comp-067---user-accounts-54267)
- [COMP-068 - User Roles](#comp-068---user-roles-54268)
- [COMP-069 - Vehicle External Environment](#comp-069---vehicle-external-environment-54269)
- [COMP-070 - Vehicle Guidance](#comp-070---vehicle-guidance-54270)
- [COMP-071 - Vehicle Performance](#comp-071---vehicle-performance-54271)
- [COMP-072 - Vehicle Stability and Control](#comp-072---vehicle-stability-and-control-54272)
- [COMP-073 - Weather](#comp-073---weather-54273)

<a id="comp-001"></a>
## COMP-001 - Anomaly Detection (5.4.2.1)
### PYR-RESP-0001 (`COMP-001-R01`) - determine_actual_state
- To interpret and correlate available System_Data to determine the Actual_State of a System_Element.

### PYR-RESP-0002 (`COMP-001-R02`) - determine_expected_state
- To interpret and correlate available System_Data to determine the Expected_State of a System_Element.

### PYR-RESP-0003 (`COMP-001-R03`) - identify_anomalies
- To identify anomalies.

## COMP-002 - Asset Transitions (5.4.2.2)
### PYR-RESP-0004 (`COMP-002-R01`) - capture_transition_requirements
- To capture given Transition_Requirements to enable a wider system capability.

### PYR-RESP-0005 (`COMP-002-R02`) - capture_measurement_criteria
- To capture given Measurement_Criterion/criteria.

### PYR-RESP-0006 (`COMP-002-R03`) - capture_transition_constraints
- To capture given Transition_Constraints on a Transition_Solution.

### PYR-RESP-0007 (`COMP-002-R04`) - determine_transition_solution
- To determine a Transition_Solution to achieve desired States.

### PYR-RESP-0008 (`COMP-002-R05`) - determine_predicted_quality_of_transition_solution
- To determine the predicted quality of a proposed Transition_Solution against the given Measurement_Criterion/criteria.

### PYR-RESP-0009 (`COMP-002-R06`) - determine_if_transition_solution_remains_feasible
- To determine the feasibility of a planned or on-going Transition_Solution.

### PYR-RESP-0010 (`COMP-002-R07`) - identify_states
- To maintain a view of the current States.

### PYR-RESP-0011 (`COMP-002-R08`) - determine_infrastructure_states
- To determine the desired States that offer the required system capability.

### PYR-RESP-0012 (`COMP-002-R09`) - implement_transition_solution
- To implement a Transition_Solution.

### PYR-RESP-0013 (`COMP-002-R10`) - identify_transition_solution_progress
- To identify the progress of a Transition_Solution against the Transition_Requirement.

### PYR-RESP-0014 (`COMP-002-R11`) - determine_actual_quality_of_transition_deliverables
- To determine the actual quality of the Transition_Solution against the given Measurement_Criterion/criteria.

### PYR-RESP-0015 (`COMP-002-R12`) - assess_capability
- To assess the Transition_Capability, taking into account available resources, system health and observed anomalies.

### PYR-RESP-0016 (`COMP-002-R13`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Transition_Capability assessment.

### PYR-RESP-0017 (`COMP-002-R14`) - predict_capability_progression
- To predict the progression of the Transition_Capability over time and with use.

## COMP-003 - Authorisation (5.4.2.3)
### PYR-RESP-0018 (`COMP-003-R01`) - capture_requirements_for_authorisations
- To capture Authorisation_Requirements.

### PYR-RESP-0019 (`COMP-003-R02`) - capture_constraints_for_authorisations
- To capture Constraints related to obtaining Authorisations.

### PYR-RESP-0020 (`COMP-003-R03`) - determine_authorisation_solution
- To determine the Steps required to obtain an Authorisation that meets the given Authorisation_Requirements, using available Authorisers in accordance with the applicable Authorisation_Policy.

### PYR-RESP-0021 (`COMP-003-R04`) - determine_authorisation_policy
- To determine the conditions under which a Authorisation_Policy will be active.

### PYR-RESP-0022 (`COMP-003-R05`) - determine_permitted_authorisers
- To determine which Authorisers are permitted (e.g. as defined in the Authorisation_Policy) to provide a given Authorisation.

### PYR-RESP-0023 (`COMP-003-R06`) - determine_if_authorisation_remains_feasible
- To determine the feasibility of a planned or on-going Authorisation.

### PYR-RESP-0024 (`COMP-003-R07`) - capture_authoriser_availability
- To capture available Authorisers (e.g. Authorisers to which or whom there is a communications link).

### PYR-RESP-0025 (`COMP-003-R08`) - coordinate_authorisation_solution
- To coordinate the execution of the Steps required to obtain Authorisation.

### PYR-RESP-0026 (`COMP-003-R09`) - identify_progress_of_authorisation
- To identify the progress of an Authorisation against the Authorisation_Requirement.

### PYR-RESP-0027 (`COMP-003-R10`) - determine_solution_dependencies
- To determine dependencies required to support Authorisation or a Step of the solution.

### PYR-RESP-0028 (`COMP-003-R11`) - assess_authorisation_capability
- To assess the Capability to obtain Authorisation taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0029 (`COMP-003-R12`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment of the Authorisation component (e.g. communication capability).

### PYR-RESP-0030 (`COMP-003-R13`) - predict_capability_progression
- To predict the progression of the Authorisation component's Capability over time and with use.

## COMP-004 - Collision Avoidance (5.4.2.4)
### PYR-RESP-0031 (`COMP-004-R01`) - capture_separation_breach
- To capture the provided Separation_Breach for which Avoidance_Measures are required.

### PYR-RESP-0032 (`COMP-004-R02`) - capture_avoidance_constraints
- To capture provided Constraints that limit the ability to avoid a collision.

### PYR-RESP-0033 (`COMP-004-R03`) - determine_avoidance_measure
- To determine the Avoidance_Measure required to avoid a collision (either cooperatively or non-cooperatively), or to exit a separation breach.

### PYR-RESP-0034 (`COMP-004-R04`) - identify_progress_of_avoidance_measure
- To identify the progress of Avoidance_Measures in addressing the Separation_Breach.

### PYR-RESP-0035 (`COMP-004-R05`) - assess_avoidance_capability
- To assess the Avoidance_Capability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0036 (`COMP-004-R06`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Avoidance_Capability assessment.

### PYR-RESP-0037 (`COMP-004-R07`) - predict_progression_of_avoidance_capability
- To predict the progression of the Avoidance_Capability over time and with use.

### PYR-RESP-0038 (`COMP-004-R08`) - identify_avoidance_measure_in_progress_remains_feasible
- To identify if the Avoidance_Measure in progress remains feasible given current Avoidance_Capability and Constraints.

## COMP-005 - Collision Prediction (5.4.2.5)
### PYR-RESP-0039 (`COMP-005-R01`) - capture_prediction_requirements
- To capture Requirements for collision prediction (e.g. during taxi, transit or air-to-air refuelling).

### PYR-RESP-0040 (`COMP-005-R02`) - capture_measurement_criteria_for_collision_prediction
- To capture provided Measurement_Criterion/criteria for collision prediction (e.g. confidence of prediction).

### PYR-RESP-0041 (`COMP-005-R03`) - determine_breach
- To determine an actual or predicted Breach.

### PYR-RESP-0042 (`COMP-005-R04`) - determine_breach_status
- To determine the status of an actual or predicted Breach, e.g. cleared or active.

### PYR-RESP-0043 (`COMP-005-R05`) - identify_rules
- To identify the rules that apply when determining if a Breach has occurred or is predicted.

### PYR-RESP-0044 (`COMP-005-R06`) - determine_quality_of_deliverables
- To determine the quality of the collision prediction, measured against given Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0045 (`COMP-005-R07`) - assess_prediction_capability
- To assess the Prediction_Capability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0046 (`COMP-005-R08`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Prediction_Capability assessment.

### PYR-RESP-0047 (`COMP-005-R09`) - predict_capability_progression
- To predict the progression of the component's Prediction_Capability over time and with use.

## COMP-006 - Communication Links (5.4.2.6)
### PYR-RESP-0048 (`COMP-006-R01`) - capture_link_requirements
- To capture given communication Link_Requirements (e.g. endpoint, throughput, reliability and latency).

### PYR-RESP-0049 (`COMP-006-R02`) - capture_link_measurement_criteria
- To capture the criteria by which Link_Quality will be measured (e.g. reliability, throughput, and latency) for Link_Solutions.

### PYR-RESP-0050 (`COMP-006-R03`) - capture_link_constraints
- To capture given communication link Constraints (e.g. maximum power level and spectrum usage).

### PYR-RESP-0051 (`COMP-006-R04`) - determine_link_solution
- To determine a communication Link_Solution (i.e. a set of Link_Options), which includes the planning and configuration of the links, that meets the given requirements and Constraints using available Link_Resources.

### PYR-RESP-0052 (`COMP-006-R05`) - determine_quality_of_link_performance_solution
- To determine the quality of a proposed communication Link_Solution against given required Link_Quality (i.e. determine the theoretical performance for a link solution).

### PYR-RESP-0053 (`COMP-006-R06`) - determine_if_link_solution_remains_feasible
- To determine the feasibility of a planned or on-going Link_Solution.

### PYR-RESP-0054 (`COMP-006-R07`) - identify_link_pre-conditions
- To identify Pre-conditions (e.g. to achieve or maintain a zone with sufficient signal visibility) to support a communication Link_Solution.

### PYR-RESP-0055 (`COMP-006-R08`) - identify_link_performance_deviation
- To identify the deviation from expected performance of a Link_Solution against given criteria.

### PYR-RESP-0056 (`COMP-006-R09`) - establish_and_maintain_links
- To establish and maintain a communication Link (including termination).

### PYR-RESP-0057 (`COMP-006-R10`) - determine_link_cost
- To determine the Link_Cost of a communication Link_Solution for a given required Link_Quality (i.e. determining the viability of a link to support a need).

### PYR-RESP-0058 (`COMP-006-R11`) - assess_link_capability
- To assess the system Capability to establish and maintain links using available Link_Resources within given Constraints.

### PYR-RESP-0059 (`COMP-006-R12`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the link management Capability assessment (e.g. observability/LoS assessment).

### PYR-RESP-0060 (`COMP-006-R13`) - predict_communication_link_capability_progression
- To predict the progression of the Communication Links component's Capability over time and with use (i.e. if the communications link is failing, provide predictions on how long the capability is capable of functioning before it fails).

## COMP-007 - Communicator (5.4.2.7)
### PYR-RESP-0061 (`COMP-007-R01`) - capture_requirements_for_communicator_resources
- To capture provided Requirements (e.g. power, latency, loss rate, direction and throughput) for the use of Communicator_Resources.

### PYR-RESP-0062 (`COMP-007-R02`) - capture_constraints_for_communicator_resources
- To capture provided Constraints for use of Communicator_Resources (e.g. maximum power or frequency).

### PYR-RESP-0063 (`COMP-007-R03`) - determine_transmission_sequence
- To determine a Transmission_Sequence for the use of Communicator_Resources that will meet given Requirements, including forward error correction, frequency migration, frequency and spatial diversity.

### PYR-RESP-0064 (`COMP-007-R04`) - identify_transmission_sequence_remains_feasible
- To identify if a Transmission_Sequence in progress remains feasible given current Communicator_Resources.

### PYR-RESP-0065 (`COMP-007-R05`) - coordinate_use_of_resources
- To coordinate the use of Communicator_Resources (e.g. determining signal direction including signal lock).

### PYR-RESP-0066 (`COMP-007-R06`) - manage_transmissions
- To manage incoming and outgoing Transmissions to an external entity, including initiation and termination (e.g. transceiver handshaking, keep-alive and timing synchronisation).

### PYR-RESP-0067 (`COMP-007-R07`) - identify_progress_of_transmission_sequence
- To identify the progress of a Communicator_Resource Transmission_Sequence against the Requirements.

### PYR-RESP-0068 (`COMP-007-R08`) - determine_quality_of_transmission_sequence
- To determine the quality of a Communicator_Resource Transmission_Sequence against a given Quality_of_Service.

### PYR-RESP-0069 (`COMP-007-R09`) - determine_quality_of_transmission
- To determine the quality of the Transmission provided by Communicator_Resources during execution, measured against the given Requirements and Quality_of_Service.

### PYR-RESP-0070 (`COMP-007-R10`) - assess_communicator_resource_capability
- To assess the Capability provided by Communicator_Resources, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0071 (`COMP-007-R11`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Communicator_Resource Capability assessment.

### PYR-RESP-0072 (`COMP-007-R12`) - predict_capability_progression
- To predict the progression of the Communicator_Resources' Capability over time and with use.

## COMP-008 - Conflict Resolution (5.4.2.8)
### PYR-RESP-0073 (`COMP-008-R01`) - capture_conflicts
- To capture requirements and requirement relationships for Conflict resolution.

### PYR-RESP-0074 (`COMP-008-R02`) - capture_constraints
- To capture Resolution_Constraints on the brokering and arbitration of Conflicts.

### PYR-RESP-0075 (`COMP-008-R03`) - determine_applicable_scheme
- To determine the applicable Scheme to apply to resolve a Conflict.

### PYR-RESP-0076 (`COMP-008-R04`) - monitor_progress
- To monitor the progress of a Conflict resolution.

### PYR-RESP-0077 (`COMP-008-R05`) - arbitrate_resource_conflicts
- To arbitrate on a Conflict in accordance with the applicable Scheme.

### PYR-RESP-0078 (`COMP-008-R06`) - coordinate_conflict_resolution
- To coordinate the activities required to broker and arbitrate a resolution to a Conflict in accordance with the applicable Scheme and Context.

### PYR-RESP-0079 (`COMP-008-R07`) - assess_capability
- To assess the Capability of the component to broker and arbitrate Conflicts.

### PYR-RESP-0080 (`COMP-008-R08`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the components capability assessment for being able to broker and arbitrate Conflicts.

## COMP-009 - Countermeasures (5.4.2.9)
### PYR-RESP-0081 (`COMP-009-R01`) - capture_countermeasure_requirements
- To capture given Countermeasure_Requirements (e.g. reduce the threat risk to an acceptable level).

### PYR-RESP-0082 (`COMP-009-R02`) - capture_measurement_criteria
- To capture given Measurement_Criterion/criteria (e.g. reduction in threat level) for countermeasure solutions.

### PYR-RESP-0083 (`COMP-009-R03`) - capture_countermeasure_constraints
- To capture given countermeasure Constraints (e.g. transmission restrictions or allowable level of response).

### PYR-RESP-0084 (`COMP-009-R04`) - identify_if_countermeasure_requirement_remains_achievable
- To identify whether a Countermeasure_Requirement is still achievable given current or predicted capability and conditions.

### PYR-RESP-0085 (`COMP-009-R05`) - identify_countermeasure_strategy_in_progress_remains_feasible
- To identify if a Countermeasure_Strategy in progress remains feasible given current Countermeasure_Resources, Constraints and external factors (e.g. changes in environmental conditions).

### PYR-RESP-0086 (`COMP-009-R06`) - identify_pre-conditions
- To identify the Pre-conditions required to support a Countermeasure_Strategy.

### PYR-RESP-0087 (`COMP-009-R07`) - co-ordinate_countermeasure_strategy
- To execute the selected Countermeasure_Strategy by commanding Countermeasure_Resources.

### PYR-RESP-0088 (`COMP-009-R08`) - identify_progress_of_countermeasure_strategy
- To identify the progress of the Countermeasure_Strategy against the Countermeasure_Requirement(s) and external factors (e.g. changes in the threat level and environmental conditions).

### PYR-RESP-0089 (`COMP-009-R09`) - determine_quality_of_deliverables
- To determine the quality of the Deliverables provided by a Countermeasure_Strategy, measured against given Countermeasure_Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0090 (`COMP-009-R10`) - assess_countermeasure_capability
- To assess the Capability to carry out a Countermeasure_Strategy or strategies using available Countermeasure_Resources, taking into account observed anomalies.

### PYR-RESP-0091 (`COMP-009-R11`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Countermeasure Capability assessment.

### PYR-RESP-0092 (`COMP-009-R12`) - predict_capability_progression
- To predict the progression of countermeasure Capability over time and with use.

<a id="comp-010"></a>
## COMP-010 - Cryptographic Materials (5.4.2.10)
### PYR-RESP-0093 (`COMP-010-R01`) - capture_crypto_material_requirements
- To capture Requirements for the use of Cryptographic_Material.

### PYR-RESP-0094 (`COMP-010-R02`) - determine_material_plan
- To determine a Material_Plan that complies with the data Segregation_Policy (e.g. for specific security domains, types or classifications of data).

### PYR-RESP-0095 (`COMP-010-R03`) - determine_crypto_material_usage
- To determine when and where particular Cryptographic_Material needs to be used.

### PYR-RESP-0096 (`COMP-010-R04`) - coordinate_sanitisation
- To coordinate the sanitisation of Cryptographic_Material, including emergency sanitisation, response to compromised key lists and certificate revocation list (CRL) checking.

### PYR-RESP-0097 (`COMP-010-R05`) - coordinate_material_change
- To coordinate the change of Cryptographic_Material, e.g. rollover of cryptographic keys and certificates to maintain its validity.

### PYR-RESP-0098 (`COMP-010-R06`) - distribute_crypto_material
- To distribute the required Cryptographic_Material.

### PYR-RESP-0099 (`COMP-010-R07`) - identify_material_solution_progress
- To identify what progress has been made against the Requirement.

### PYR-RESP-0100 (`COMP-010-R08`) - assess_capability
- To assess the ability of the component to provide appropriate Cryptographic_Material, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0101 (`COMP-010-R09`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

## COMP-011 - Cryptographic Methods (5.4.2.11)
### PYR-RESP-0102 (`COMP-011-R01`) - capture_cryptographic_requirement
- To capture Requirements for a Cryptographic_Function (e.g. encryption, decryption, or hashing).

### PYR-RESP-0103 (`COMP-011-R02`) - determine_cryptographic_material_for_use
- To determine the Cryptographic_Material to be used for any particular Cryptographic_Action.

### PYR-RESP-0104 (`COMP-011-R03`) - identify_if_cryptographic_transformation_solution_remains_feasible
- To identify if a cryptographic transformation in progress remains feasible given current resources.

### PYR-RESP-0105 (`COMP-011-R04`) - determine_cryptographic_state
- To determine the current state of the cryptography, e.g. encryption is available, complete, or failed.

### PYR-RESP-0106 (`COMP-011-R05`) - encrypt_data
- To encrypt data.

### PYR-RESP-0107 (`COMP-011-R06`) - decrypt_data
- To decrypt data.

### PYR-RESP-0108 (`COMP-011-R07`) - provide_hashing_function
- To hash data.

### PYR-RESP-0109 (`COMP-011-R08`) - capture_cryptographic_material
- To capture provided Cryptographic_Material.

### PYR-RESP-0110 (`COMP-011-R09`) - assess_cryptographic_capability
- To assess the Capability of the component taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0111 (`COMP-011-R10`) - sanitise_cryptographic_material
- To sanitise provided Cryptographic_Material.

## COMP-012 - Cyber Defence (5.4.2.12)
### PYR-RESP-0112 (`COMP-012-R01`) - determine_possible_actions
- To identify possible actions to counteract a suspected cyber attack.

### PYR-RESP-0113 (`COMP-012-R02`) - determine_anomaly_cause
- To determine that anomalous system behaviour may be the result of a cyber attack.

### PYR-RESP-0114 (`COMP-012-R03`) - identify_affected_system_elements
- To identify System_Elements that have been affected by a suspected cyber attack.

### PYR-RESP-0115 (`COMP-012-R04`) - predict_cyber_attack_progression
- To predict the progression of a cyber attack through the system (i.e. the expected sequence in which System_Elements are likely to be affected).

### PYR-RESP-0116 (`COMP-012-R05`) - determine_quality_of_identification
- To determine the quality of a cyber attack determination, against given Measurement_Criterion/criteria.

### PYR-RESP-0117 (`COMP-012-R06`) - determine_quality_of_response
- To determine the quality of a Response to a cyber attack, against given Measurement_Criterion/criteria.

### PYR-RESP-0118 (`COMP-012-R07`) - identify_additional_evidence_to_improve_identification
- To identify additional Evidence that could improve the certainty or specificity of a cyber attack determination.

## COMP-013 - Data Distribution (5.4.2.13)
### PYR-RESP-0119 (`COMP-013-R01`) - capture_requirements_for_data_distribution
- To capture Distribution_Requirements for distribution of Delivery_Items.

### PYR-RESP-0120 (`COMP-013-R02`) - capture_measurement_criteria_for_data_distribution
- To capture Measurement_Criteria for distribution of a Delivery_Item.

### PYR-RESP-0121 (`COMP-013-R03`) - capture_constraints_for_data_distribution
- To capture Distribution_Constraints for the distribution of Delivery_Items.

### PYR-RESP-0122 (`COMP-013-R04`) - determine_data_distribution_solution
- To determine a data distribution solution (e.g. transport method and protocol or report formatting) for use of Delivery_Resources that will meet given Distribution_Requirements, Distribution_Constraints and Measurement_Criteria.

### PYR-RESP-0123 (`COMP-013-R05`) - gather_data_for_distribution
- To gather a Data_Item for distribution to a Participant in accordance with an agreed data distribution solution using Delivery_Resources.

### PYR-RESP-0124 (`COMP-013-R06`) - format_data_for_distribution
- To format a Delivery_Item using the specified Formatting_Rules for distribution to a Participant in accordance with an agreed data distribution solution.

### PYR-RESP-0125 (`COMP-013-R07`) - protect_data_for_distribution
- To protect a Delivery_Item using the specified Protections for distribution to a Participant in accordance with an agreed data distribution solution.

### PYR-RESP-0126 (`COMP-013-R08`) - identify_data_distribution_solution_in_progress_remains_feasible
- To identify whether a data distribution solution currently in progress remains feasible.

### PYR-RESP-0127 (`COMP-013-R09`) - distribute_data
- To distribute a Delivery_Item in accordance with an agreed data distribution solution using Delivery_Resources.

### PYR-RESP-0128 (`COMP-013-R10`) - identify_progress_of_data_distribution_solution
- To identify the progress of a data distribution solution against the captured Distribution_Requirements.

### PYR-RESP-0129 (`COMP-013-R11`) - determine_quality_of_data_distribution
- To determine the quality of a Delivery_Interaction, measured against given Distribution_Requirements and Measurement_Criteria (e.g. for data delivery this could be loss of packets or corrupted data).

### PYR-RESP-0130 (`COMP-013-R12`) - assess_data_distribution_capability
- To assess the Distribution_Capability to distribute Delivery_Items taking account of system health and observed anomalies.

### PYR-RESP-0131 (`COMP-013-R13`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Distribution_Capability assessment.

### PYR-RESP-0132 (`COMP-013-R14`) - predict_capability_progression
- To predict the progression of the Distribution_Capability over time and with use.

## COMP-014 - Data Fusion (5.4.2.14)
### PYR-RESP-0133 (`COMP-014-R01`) - capture_fusion_requirements
- To capture Data_Fusion_Requirements for interpreting Evidence and generating Fused_Objects.

### PYR-RESP-0134 (`COMP-014-R02`) - capture_measurement_criteria_for_fusion
- To capture provided Measurement_Criterion (e.g. timeliness, confidence, completeness or accuracy) for data fusion.

### PYR-RESP-0135 (`COMP-014-R03`) - capture_fusion_constraints
- To capture Fusion_Constraints for data fusion (e.g. restriction on sources of Evidence).

### PYR-RESP-0136 (`COMP-014-R04`) - determine_if_fusion_requirement_is_achievable
- To determine if a Data_Fusion_Requirement is achievable, given current Fusion_Constraints and resources.

### PYR-RESP-0137 (`COMP-014-R05`) - determine_fusion_solution
- To determine how to meet the given Data_Fusion_Requirements, within applicable Fusion_Constraints.

### PYR-RESP-0138 (`COMP-014-R06`) - determine_predicted_quality_of_fusion_solution
- To determine the predicted quality of a data fusion solution against given Measurement_Criterion.

### PYR-RESP-0139 (`COMP-014-R07`) - generate_fused_objects
- To generate Fused_Objects based on interpretation of the available Evidence.

### PYR-RESP-0140 (`COMP-014-R08`) - maintain_fused_object_lineage
- To maintain the lineage of Fused_Objects (e.g. lineage between Evidence and Fused_Objects, and the merging and splitting Fused_Objects).

### PYR-RESP-0141 (`COMP-014-R09`) - capture_evidence
- To capture provided Evidence along with its lineage.

### PYR-RESP-0142 (`COMP-014-R10`) - capture_supporting_information
- To capture provided Supporting_Information (e.g. platform data, weather condition or terrain data).

### PYR-RESP-0143 (`COMP-014-R11`) - determine_quality_of_fused_objects
- To determine the quality of the Fused_Objects, measured against given Data_Fusion_Requirements and Measurement_Criterion.

### PYR-RESP-0144 (`COMP-014-R12`) - assess_capability
- To assess the capability to generate Fused_Objects, taking into account observed anomalies.

### PYR-RESP-0145 (`COMP-014-R13`) - identify_missing_capability_information
- To identify missing information that could improve the certainty or specificity of the Data Fusion capability assessment.

### PYR-RESP-0146 (`COMP-014-R14`) - predict_capability_progression
- To predict the progression of Fusion_Capability over time and with use, e.g. Data Fusion determines that its Fusion_Capability is downgraded due to the intermittent availability of an Evidence_Type.

### PYR-RESP-0147 (`COMP-014-R15`) - Evidence_Lineage
- Description not extracted from source text.

### PYR-RESP-0148 (`COMP-014-R16`) - Fused_Object_Quality
- Description not extracted from source text.

### PYR-RESP-0149 (`COMP-014-R17`) - Related PYRAMID Concepts
- Description not extracted from source text.

### PYR-RESP-0150 (`COMP-014-R18`) - Extensions
- Description not extracted from source text.

### PYR-RESP-0151 (`COMP-014-R19`) - Fusion_Requirement
- Description not extracted from source text.

### PYR-RESP-0152 (`COMP-014-R20`) - Interfaces
- Description not extracted from source text.

### PYR-RESP-0153 (`COMP-014-R21`) - Data_Fusion_Requirement
- Description not extracted from source text.

### PYR-RESP-0154 (`COMP-014-R22`) - Data_Fusion_Criterion
- Description not extracted from source text.

### PYR-RESP-0155 (`COMP-014-R23`) - Fused_Object_Achievement
- Description not extracted from source text.

### PYR-RESP-0156 (`COMP-014-R24`) - Activities
- Description not extracted from source text.

### PYR-RESP-0157 (`COMP-014-R25`) - determine_data_fusion_solution
- Description not extracted from source text.

### PYR-RESP-0158 (`COMP-014-R26`) - execute_data_fusion_solution
- Description not extracted from source text.

### PYR-RESP-0159 (`COMP-014-R27`) - identify_whether_data_fusion_requirement_is_achievable
- Description not extracted from source text.

### PYR-RESP-0160 (`COMP-014-R28`) - Figure 242: Fused_Object_Information Service Definition
- Description not extracted from source text.

### PYR-RESP-0161 (`COMP-014-R29`) - Figure 243: Fused_Object_Information Service Policy
- Description not extracted from source text.

### PYR-RESP-0162 (`COMP-014-R30`) - Fused_Object_Information
- Description not extracted from source text.

### PYR-RESP-0163 (`COMP-014-R31`) - Interfaces
- Description not extracted from source text.

### PYR-RESP-0164 (`COMP-014-R32`) - Traceability_Information
- Description not extracted from source text.

### PYR-RESP-0165 (`COMP-014-R33`) - Fused_Object_Information
- Description not extracted from source text.

### PYR-RESP-0166 (`COMP-014-R34`) - Activity
- Description not extracted from source text.

### PYR-RESP-0167 (`COMP-014-R35`) - determine_fused_object_information
- Description not extracted from source text.

### PYR-RESP-0168 (`COMP-014-R36`) - Evidence
- Description not extracted from source text.

### PYR-RESP-0169 (`COMP-014-R37`) - Interface
- Description not extracted from source text.

### PYR-RESP-0170 (`COMP-014-R38`) - Evidence_Information
- Description not extracted from source text.

### PYR-RESP-0171 (`COMP-014-R39`) - Activities
- Description not extracted from source text.

### PYR-RESP-0172 (`COMP-014-R40`) - identify_required_evidence_information
- Description not extracted from source text.

### PYR-RESP-0173 (`COMP-014-R41`) - assess_evidence_information_update
- Description not extracted from source text.

### PYR-RESP-0174 (`COMP-014-R42`) - Figure 246: Environmental_Data Service Definition
- Description not extracted from source text.

### PYR-RESP-0175 (`COMP-014-R43`) - Environmental_Data
- Description not extracted from source text.

### PYR-RESP-0176 (`COMP-014-R44`) - Interface
- Description not extracted from source text.

### PYR-RESP-0177 (`COMP-014-R45`) - Environmental_Data
- Description not extracted from source text.

### PYR-RESP-0178 (`COMP-014-R46`) - Activities
- Description not extracted from source text.

### PYR-RESP-0179 (`COMP-014-R47`) - assess_environmental_data_update
- Description not extracted from source text.

### PYR-RESP-0180 (`COMP-014-R48`) - identify_required_environmental_data
- Description not extracted from source text.

### PYR-RESP-0181 (`COMP-014-R49`) - Figure 248: Vehicle_Data Service Definition
- Description not extracted from source text.

### PYR-RESP-0182 (`COMP-014-R50`) - Vehicle_Data
- Description not extracted from source text.

### PYR-RESP-0183 (`COMP-014-R51`) - Interface
- Description not extracted from source text.

### PYR-RESP-0184 (`COMP-014-R52`) - Vehicle_Data
- Description not extracted from source text.

### PYR-RESP-0185 (`COMP-014-R53`) - Activities
- Description not extracted from source text.

### PYR-RESP-0186 (`COMP-014-R54`) - assess_vehicle_data_update
- Description not extracted from source text.

### PYR-RESP-0187 (`COMP-014-R55`) - identify_required_vehicle_data
- Description not extracted from source text.

### PYR-RESP-0188 (`COMP-014-R56`) - Object_Data
- Description not extracted from source text.

### PYR-RESP-0189 (`COMP-014-R57`) - Interface
- Description not extracted from source text.

### PYR-RESP-0190 (`COMP-014-R58`) - Object_Data
- Description not extracted from source text.

### PYR-RESP-0191 (`COMP-014-R59`) - Activities
- Description not extracted from source text.

### PYR-RESP-0192 (`COMP-014-R60`) - assess_object_data_update
- Description not extracted from source text.

### PYR-RESP-0193 (`COMP-014-R61`) - identify_required_object_data
- Description not extracted from source text.

### PYR-RESP-0194 (`COMP-014-R62`) - Figure 252: Constraint Service Definition
- Description not extracted from source text.

### PYR-RESP-0195 (`COMP-014-R63`) - Constraint
- Description not extracted from source text.

### PYR-RESP-0196 (`COMP-014-R64`) - Interface
- Description not extracted from source text.

### PYR-RESP-0197 (`COMP-014-R65`) - Evidence_Constraints
- Description not extracted from source text.

### PYR-RESP-0198 (`COMP-014-R66`) - Activities
- Description not extracted from source text.

### PYR-RESP-0199 (`COMP-014-R67`) - evaluate_impact_of_fusion_constraint
- Description not extracted from source text.

### PYR-RESP-0200 (`COMP-014-R68`) - identify_required_context
- Description not extracted from source text.

### PYR-RESP-0201 (`COMP-014-R69`) - Interface
- Description not extracted from source text.

### PYR-RESP-0202 (`COMP-014-R70`) - Fusion_Capability
- Description not extracted from source text.

### PYR-RESP-0203 (`COMP-014-R71`) - Activity
- Description not extracted from source text.

### PYR-RESP-0204 (`COMP-014-R72`) - determine_fusion_capability
- Description not extracted from source text.

### PYR-RESP-0205 (`COMP-014-R73`) - Fusion_Capability_Evidence
- Description not extracted from source text.

### PYR-RESP-0206 (`COMP-014-R74`) - Interfaces
- Description not extracted from source text.

### PYR-RESP-0207 (`COMP-014-R75`) - Evidence_Source_Capability
- Description not extracted from source text.

### PYR-RESP-0208 (`COMP-014-R76`) - Vehicle_Data_Source_Capability
- Description not extracted from source text.

### PYR-RESP-0209 (`COMP-014-R77`) - Object_Data_Source_Capability
- Description not extracted from source text.

### PYR-RESP-0210 (`COMP-014-R78`) - Environmental_Data_Source_Capability
- Description not extracted from source text.

### PYR-RESP-0211 (`COMP-014-R79`) - Activities
- Description not extracted from source text.

### PYR-RESP-0212 (`COMP-014-R80`) - assess_data_fusion_capability_evidence
- Description not extracted from source text.

### PYR-RESP-0213 (`COMP-014-R81`) - identify_missing_data_fusion_capability_evidence
- Description not extracted from source text.

### PYR-RESP-0214 (`COMP-014-R82`) - Control Architecture
- Description not extracted from source text.

### PYR-RESP-0215 (`COMP-014-R83`) - Standard Pattern of Use
- Description not extracted from source text.

### PYR-RESP-0216 (`COMP-014-R84`) - Examples of Use
- Description not extracted from source text.

## COMP-015 - Destructive Effects (5.4.2.15)
### PYR-RESP-0217 (`COMP-015-R01`) - capture_requirements_for_destructive_effects
- To capture provided Destructive_Effect_Requirements.

### PYR-RESP-0218 (`COMP-015-R02`) - capture_measurement_criteria_for_destructive_effects
- To capture provided Measurement_Criterion for the use of the Destructive_Effects.

### PYR-RESP-0219 (`COMP-015-R03`) - capture_constraints_for_destructive_effects
- To capture provided Constraints that apply to the use of Weapon_Resources.

### PYR-RESP-0220 (`COMP-015-R04`) - identify_whether_requirement_is_achievable
- To identify whether a Destructive_Effect_Requirement is achievable given current Weapon_Resources, Destructive_Effect_Settings and External_Influences.

### PYR-RESP-0221 (`COMP-015-R05`) - determine_destructive_effect
- To determine a Destructive_Effect that will meet given Destructive_Effect_Requirements, and identify the associated Weapon_Type and Destructive_Effect_Settings.

### PYR-RESP-0222 (`COMP-015-R06`) - identify_pre-conditions
- To identify Pre-conditions required to achieve a Destructive_Effect.

### PYR-RESP-0223 (`COMP-015-R07`) - control_destructive_effect_settings
- To control the Destructive_Effect_Settings, e.g. set fusing mode.

### PYR-RESP-0224 (`COMP-015-R08`) - determine_quality_of_destructive_effects_solution
- To determine the quality of a Destructive_Effect against given Measurement_Criterion.

### PYR-RESP-0225 (`COMP-015-R09`) - determine_destructive_effects_capability
- To assess the Destructive_Effect_Capability, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0226 (`COMP-015-R10`) - identify_required_information
- To identify missing information which could improve the certainty or specificity of the capability assessment for Destructive Effects.

### PYR-RESP-0227 (`COMP-015-R11`) - predict_capability_progression
- To predict the progression of the Destructive_Effect_Capability, over time and with use.

## COMP-016 - Effectors (5.4.2.16)
### PYR-RESP-0228 (`COMP-016-R01`) - capture_requirements_for_effector_resources
- To capture provided requirements for use of Effector_Resources (e.g. turn valve off now, select flap to position 3 in 2 seconds or increase output voltage at 3V per second for 6 seconds).

### PYR-RESP-0229 (`COMP-016-R02`) - capture_measurement_criteria
- To capture Measurement_Criterion for an Effect.

### PYR-RESP-0230 (`COMP-016-R03`) - capture_effector_constraints
- To capture provided constraints, e.g. EMCON.

### PYR-RESP-0231 (`COMP-016-R04`) - determine_effector_solution
- To determine a solution for use of Effector_Resources that will meet given Requirements.

### PYR-RESP-0232 (`COMP-016-R05`) - determine_resources_used_by_the_effector
- To determine information about effectors use of the resources.

### PYR-RESP-0233 (`COMP-016-R06`) - determine_if_solution_remains_feasible
- To determine if a planned or ongoing Effector_Solution remains feasible.

### PYR-RESP-0234 (`COMP-016-R07`) - control_use_of_effector
- To control the use of Effector_Resources.

### PYR-RESP-0235 (`COMP-016-R08`) - identify_progress_of_effector
- To identify the progress of the Effector_Solution use against the Requirement.

### PYR-RESP-0236 (`COMP-016-R09`) - assess_effector_capability
- To assess the Capability provided by Effector_Resources, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0237 (`COMP-016-R10`) - predict_capability_progression
- To predict the progression of Effector_Resource capability over time and with use.

## COMP-017 - Environment Infrastructure (5.4.2.17)
### PYR-RESP-0238 (`COMP-017-R01`) - capture_infrastructure_information_request
- To capture the requests for information on Infrastructure_Features, the relationship between them, or the relationship between a Reference_Item and Infrastructure_Features (e.g. a request to identify recovery locations, or suitable beacons).

### PYR-RESP-0239 (`COMP-017-R02`) - determine_CTT_forced_landing_locations
- To determine available Controlled-Trajectory Termination (CTT) or forced landing locations.

### PYR-RESP-0240 (`COMP-017-R03`) - determine_terminal_operation_areas
- To determine available Terminal_Operation_Areas, including those which are not formally designated by authorities.

### PYR-RESP-0241 (`COMP-017-R04`) - determine_infrastructure_feature_properties
- To determine the properties of Infrastructure_Features.

### PYR-RESP-0242 (`COMP-017-R05`) - determine_navigation_aids
- To determine available aids to navigation.

### PYR-RESP-0243 (`COMP-017-R06`) - determine_minimum_safe_altitude
- To determine Minimum Safe Altitude (MSA) levels.

### PYR-RESP-0244 (`COMP-017-R07`) - determine_TOA_profiles
- To determine the available profiles to support taxi, launch and recovery within a Terminal_Operation_Area.

### PYR-RESP-0245 (`COMP-017-R08`) - determine_reference_relationship
- To determine the relationship between a Reference_Item (e.g. ownship position) and Infrastructure_Features.

### PYR-RESP-0246 (`COMP-017-R09`) - determine_infrastructure_conflict
- To determine when a Reference_Item (e.g. ownship's projected route) conflicts with an Infrastructure_Feature (e.g. path enters no-fly zone or breaches MSA).

### PYR-RESP-0247 (`COMP-017-R10`) - determine_feature_relationship
- To determine information on how Infrastructure_Features relate to each other.

### PYR-RESP-0248 (`COMP-017-R11`) - assess_capability
- To assess the Infrastructure_Capability to provide infrastructure information taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing such as loss of an infrastructure data resource).

### PYR-RESP-0249 (`COMP-017-R12`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Infrastructure_Capability assessment (e.g. identifying that information from an infrastructure data source may not have been updated).

## COMP-018 - Environment Integration (5.4.2.18)
### PYR-RESP-0250 (`COMP-018-R01`) - capture_requirements_for_environment_integration_actions
- To capture provided Requirements (e.g. coordinate ATC transition) for environment integration actions.

### PYR-RESP-0251 (`COMP-018-R02`) - capture_measurement_criteria_for_environment_integration_actions
- To capture provided Measurement_Criterion/criteria that an Integration_Solution and its Outcomes will be measured against.

### PYR-RESP-0252 (`COMP-018-R03`) - capture_environment_integration_constraints
- To capture provided Constraints for environment integration actions.

### PYR-RESP-0253 (`COMP-018-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current or predicted Integration_Capability and Constraints.

### PYR-RESP-0254 (`COMP-018-R05`) - determine_environment_integration_solution
- To determine an Integration_Solution that meets the given Requirements within provided Constraints using the available System_Capability.

### PYR-RESP-0255 (`COMP-018-R06`) - determine_predicted_quality_of_environment_integration_solution
- To determine the predicted quality of the Integration_Solution against given Measurement_Criterion/criteria.

### PYR-RESP-0256 (`COMP-018-R07`) - determine_integration_settings
- To determine the Integration_Settings for the operating environment.

### PYR-RESP-0257 (`COMP-018-R08`) - determine_applicable_environment_integration_rules
- To determine the currently applicable Protocols for integrating into the operating environment.

### PYR-RESP-0258 (`COMP-018-R09`) - identify_environment_integration_pre_conditions
- To identify Pre-conditions required to support the Integration_Solution or an Action_Step of the Integration_Solution.

### PYR-RESP-0259 (`COMP-018-R10`) - coordinate_environment_integration_solution
- To coordinate the execution of an Integration_Solution.

### PYR-RESP-0260 (`COMP-018-R11`) - coordinate_interactions_with_controlling_service
- To generate and interpret automated interactions with the Controlling_Service.

### PYR-RESP-0261 (`COMP-018-R12`) - identify_progress_of_environment_integration_solution
- To identify the progress of an Integration_Solution against the Requirements.

### PYR-RESP-0262 (`COMP-018-R13`) - determine_actual_quality_of_environment_integration_deliverables
- To determine the actual quality of the Integration_Solution against given Measurement_Criterion/criteria.

### PYR-RESP-0263 (`COMP-018-R14`) - provide_integration_solution_information
- To provide information relating to an Integration_Solution.

### PYR-RESP-0264 (`COMP-018-R15`) - assess_environment_integration_capability
- To assess the Integration_Capability to perform environment integration actions (e.g. respond to ATC instruction) taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0265 (`COMP-018-R16`) - predict_capability_progression
- To predict the progression of Environment Integration's Integration_Capability over time and with use.

## COMP-019 - Environmental Conditioning (5.4.2.19)
### PYR-RESP-0266 (`COMP-019-R01`) - capture_zone_requirements
- To capture given Zone_Requirements for an Environmental_Zone (e.g. a required temperature range).

### PYR-RESP-0267 (`COMP-019-R02`) - capture_measurement_criteria
- To capture provided Measurement_Criterion for each Zone_Requirement (e.g. response time).

### PYR-RESP-0268 (`COMP-019-R03`) - capture_constraints
- To capture provided Constraints for Conditioning_Mechanisms (e.g. heating of an Environmental_Zone is not permitted whilst a door is open).

### PYR-RESP-0269 (`COMP-019-R04`) - determine_solution
- To determine a Conditioning_Procedure that meets Zone_Requirements and Constraints using a Conditioning_Mechanism.

### PYR-RESP-0270 (`COMP-019-R05`) - identify_solution_in_progress_remains_feasible
- To identify whether a Conditioning_Procedure in progress remains feasible given current resources.

### PYR-RESP-0271 (`COMP-019-R06`) - coordinate_solution
- To coordinate the execution of a Conditioning_Procedure via the use of Conditioning_Mechanisms.

### PYR-RESP-0272 (`COMP-019-R07`) - identify_progress_of_solution
- To identify the progress of a Conditioning_Procedure against the Zone_Requirements.

### PYR-RESP-0273 (`COMP-019-R08`) - determine_quality_of_solution
- To determine the quality of a proposed Conditioning_Procedure against given Measurement_Criterion or criteria.

### PYR-RESP-0274 (`COMP-019-R09`) - determine_quality_of_deliverables
- To determine the quality of the Environmental_Property or properties controlled by executing a Conditioning_Procedure, measured against given Zone_Requirements and Measurement_Criterion or criteria.

### PYR-RESP-0275 (`COMP-019-R10`) - assess_environmental_conditioning_capability
- To assess capability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0276 (`COMP-019-R11`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the capability assessment.

### PYR-RESP-0277 (`COMP-019-R12`) - predict_capability_progression
- To predict the progression of capability over time and with use.

<a id="comp-020"></a>
## COMP-020 - Flights (5.4.2.20)
### PYR-RESP-0278 (`COMP-020-R01`) - capture_requirements
- To capture provided Requirements related to Flight membership (including requests to join or leave a Flight and the rules and assessments applicable to joining).

### PYR-RESP-0279 (`COMP-020-R02`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current or predicted Capability.

### PYR-RESP-0280 (`COMP-020-R03`) - coordinate_flight_member_departure
- To coordinate departure of a Member from a Flight.

### PYR-RESP-0281 (`COMP-020-R04`) - coordinate_flight_member_arrival
- To coordinate arrival of a Member into a Flight.

### PYR-RESP-0282 (`COMP-020-R05`) - coordinate_role_handover
- To coordinate a pre-planned Role handover to an eligible Member.

### PYR-RESP-0283 (`COMP-020-R06`) - coordinate_role_takeover
- To coordinate an unplanned Role takeover due to loss of capability suffered by a Member.

### PYR-RESP-0284 (`COMP-020-R07`) - maintain_flight_control_structure
- To manage the Flight in accordance with the Control_Structure (e.g. maintain the hierarchy).

### PYR-RESP-0285 (`COMP-020-R08`) - identify_achievement
- To identify what has been achieved against the Requirement.

### PYR-RESP-0286 (`COMP-020-R09`) - identify_flight_members
- To identify the Members that make up a Flight.

### PYR-RESP-0287 (`COMP-020-R10`) - identify_role_absence
- To identify when a Role is no longer being fulfilled (e.g. the flight lead is not capable of staying in command due to being destroyed or damaged).

### PYR-RESP-0288 (`COMP-020-R11`) - assess_capability
- To assess the Capability to manage Flight composition taking account of system health and observed anomalies.

### PYR-RESP-0289 (`COMP-020-R12`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment (e.g. identifying that the status of a Member is not being updated).

### PYR-RESP-0290 (`COMP-020-R13`) - predict_capability_progression
- To predict the progression of Flights' Capability over time and with use (e.g. predicting that capability will reduce because communication transmission capacity is deteriorating).

## COMP-021 - Fluids (5.4.2.21)
### PYR-RESP-0291 (`COMP-021-R01`) - capture_transfer_requirements
- To capture Transfer_Requirements to transfer fluid.

### PYR-RESP-0292 (`COMP-021-R02`) - capture_storage_requirements
- To capture Storage_Requirements to store fluid.

### PYR-RESP-0293 (`COMP-021-R03`) - capture_fluid_management_criteria
- To capture provided measurement criteria for fluid management.

### PYR-RESP-0294 (`COMP-021-R04`) - capture_constraints
- To capture Constraints related to fluid storage and transfer.

### PYR-RESP-0295 (`COMP-021-R05`) - update_achievability
- To identify whether a Fluid_Management_Requirement is still achievable given current or predicted Capability and Constraints.

### PYR-RESP-0296 (`COMP-021-R06`) - determine_transfer_solution
- To determine a fluid transfer solution.

### PYR-RESP-0297 (`COMP-021-R07`) - determine_storage_solution
- To determine a fluid storage solution.

### PYR-RESP-0298 (`COMP-021-R08`) - determine_predicted_fluid_management_quality
- To determine the predicted quality of a proposed Fluid_Management_Solution against given measurement criteria.

### PYR-RESP-0299 (`COMP-021-R09`) - identify_distribution_pre-conditions
- To identify Vehicle_States required to support fluid distribution.

### PYR-RESP-0300 (`COMP-021-R10`) - coordinate_fluid_storage
- To coordinate a solution for the storage of fluid.

### PYR-RESP-0301 (`COMP-021-R11`) - coordinate_fluid_transfer
- To coordinate a solution for the transfer of fluid.

### PYR-RESP-0302 (`COMP-021-R12`) - identify_fluid_distribution_progress
- To identify the progress of a fluid distribution solution against a Fluid_Management_Requirement.

### PYR-RESP-0303 (`COMP-021-R13`) - determine_actual_fluid_management_quality
- To determine the actual quality of a proposed Fluid_Management_Solution against given measurement criteria.

### PYR-RESP-0304 (`COMP-021-R14`) - monitor_fluid_properties
- To monitor the properties of fluid (e.g. quantity or temperature).

### PYR-RESP-0305 (`COMP-021-R15`) - assess_fluid_management_capability
- To assess the Capability to store and transfer fluid, taking account of system health and anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0306 (`COMP-021-R16`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0307 (`COMP-021-R17`) - predict_capability_progression
- To predict the progression of Capability over time and with use.

## COMP-022 - Formations (5.4.2.22)
### PYR-RESP-0308 (`COMP-022-R01`) - capture_formation_requirements
- To capture given formation Requirements (e.g. number of Formation_Members, required positional relationships between Formation_Members, and lead vehicle identification).

### PYR-RESP-0309 (`COMP-022-R02`) - capture_measurement_criteria_for_formations
- To capture provided Measurement_Criterion/criteria for Formation_Solutions and Delivered_Formations.

### PYR-RESP-0310 (`COMP-022-R03`) - capture_formation_constraints
- To capture given Constraints affecting formations (e.g. EMCON).

### PYR-RESP-0311 (`COMP-022-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current or predicted Capability and Constraints.

### PYR-RESP-0312 (`COMP-022-R05`) - determine_formation_solution
- To determine a Formation_Solution that meets the given Requirements, within Constraints and Formation_Member Capability.

### PYR-RESP-0313 (`COMP-022-R06`) - determine_predicted_quality_of_formation_solution
- To determine the predicted quality of a proposed Formation_Solution against given Measurement_Criterion/criteria.

### PYR-RESP-0314 (`COMP-022-R07`) - identify_pre-conditions
- To identify Pre-conditions in support of a Formation_Solution.

### PYR-RESP-0315 (`COMP-022-R08`) - coordinate_formation_solution
- To execute an agreed Formation_Solution by coordinating the positions of Controllable_Vehicles.

### PYR-RESP-0316 (`COMP-022-R09`) - identify_progress_of_formation_solution
- To identify the progress of a Formation_Solution against the given Requirements.

### PYR-RESP-0317 (`COMP-022-R10`) - determine_actual_quality_of_deliverables
- To determine the quality of the Delivered_Formation provided by a solution, measured against given Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0318 (`COMP-022-R11`) - assess_formation_capability
- To assess the Capability to plan and execute Formation_Solutions taking account of Formation_Members' health and observed anomalies.

### PYR-RESP-0319 (`COMP-022-R12`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0320 (`COMP-022-R13`) - predict_capability_progression
- To predict the progression of Formations Capability over time and with use.

## COMP-023 - Geography (5.4.2.23)
### PYR-RESP-0321 (`COMP-023-R01`) - capture_geography_information_request
- To capture a request for information on Geographical_Features, the relationship between them, or the relationship between a Reference_Item and Geographical_Feature.

### PYR-RESP-0322 (`COMP-023-R02`) - determine_characteristics
- To determine information about a Geographical_Feature (e.g. location, whether a terrain surface is wooded or rocky, or the magnetic variation of a location).

### PYR-RESP-0323 (`COMP-023-R03`) - determine_feature_relationships
- To determine information on how Geographical_Features relate to one another (e.g. the distance between two Geographical_Features or what other Geographical_Features exist within a zonal feature).

### PYR-RESP-0324 (`COMP-023-R04`) - determine_terrain_conflict
- To determine when a Reference_Item (e.g. ownship's projected route) conflicts with a Geographical_Feature.

### PYR-RESP-0325 (`COMP-023-R05`) - determine_reference_relationship
- To determine information on the relationship between a Reference_Item (e.g. ownship position) and Geographical_Features.

### PYR-RESP-0326 (`COMP-023-R06`) - assess_geography_service_capability
- To assess the Capability to provide information on Geographical_Features, relationships between Geographical_Features, and relationships between a Reference_Item and a Geographical_Feature.

### PYR-RESP-0327 (`COMP-023-R07`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

## COMP-024 - Health Assessment (5.4.2.24)
### PYR-RESP-0328 (`COMP-024-R01`) - predict_degradation
- To predict the progression of Hardware degradation over time and with use.

### PYR-RESP-0329 (`COMP-024-R02`) - determine_health_change
- To determine if a health change (degradation or improvement) has caused anomalous system behaviour.

### PYR-RESP-0330 (`COMP-024-R03`) - determine_life_consumed
- To identify the extent to which the Hardware's life has been consumed (calendar time remaining to next service, etc.).

### PYR-RESP-0331 (`COMP-024-R04`) - determine_usage
- To identify how much the Hardware has been used (flying hours used, hard landings experienced, etc.).

### PYR-RESP-0332 (`COMP-024-R05`) - identify_extent_of_degradation
- To identify the extent to which Hardware has been degraded by Failure, Damage, Usage or ageing.

### PYR-RESP-0333 (`COMP-024-R06`) - identify_missing_information_to_improve_health_solution
- To identify Missing_Health_Information which could improve the certainty or specificity of the health assessment.

## COMP-025 - HMI Dialogue (5.4.2.25)
### PYR-RESP-0334 (`COMP-025-R01`) - capture_dialogue_requirements
- To capture Requirements for provision of Presentable_Information to a consumer Participant. For example, a requirement for a user request to be interpreted and information provided to the appropriate system elements; or for information from multiple components to be processed together so as to be understood by a user.

### PYR-RESP-0335 (`COMP-025-R02`) - capture_provided_data_constraints
- To capture Constraints on the provided data that limit the extent of possible dialogue.

### PYR-RESP-0336 (`COMP-025-R03`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current or predicted Capability, Constraints, Context and progress of the Interaction_Sequence.

### PYR-RESP-0337 (`COMP-025-R04`) - determine_comprehension_rules
- To determine Comprehension_Rules for Source_Data in the context of the associated dialogue, within any Constraints.

### PYR-RESP-0338 (`COMP-025-R05`) - request_participant_interaction
- To request Participants to perform a Dialogue_Interaction which provides additional Source_Data or Context to support a dialogue.

### PYR-RESP-0339 (`COMP-025-R06`) - identify_interaction_pre-conditions
- To identify Pre-conditions required for a Dialogue_Interaction.

### PYR-RESP-0340 (`COMP-025-R07`) - fulfil_dialogue_requirement
- To derive Presentable_Information in support of dialogue.

### PYR-RESP-0341 (`COMP-025-R08`) - identify_dialogue_progress
- To identify progress of an Interaction_Sequence and how it relates to achievement against the Requirement. For example, the effect of a halt in user input or system data provision.

### PYR-RESP-0342 (`COMP-025-R09`) - capture_information_dependencies
- To capture information dependencies for Dialogue_Interactions, such as a Context change or the condition of Participants.

### PYR-RESP-0343 (`COMP-025-R10`) - capture_participant_relationships
- To capture Participant relationships and associated Comprehension_Rules.

### PYR-RESP-0344 (`COMP-025-R11`) - assess_dialogue_capability
- To assess the component's Capability to provide a Dialogue_Interaction, taking account of system health and observed anomalies.

### PYR-RESP-0345 (`COMP-025-R12`) - identify_missing_information
- To identify missing information, including Context, which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0346 (`COMP-025-R13`) - predict_capability_progression
- To predict the progression of the component's Capability over time and with use.

## COMP-026 - Human Interaction (5.4.2.26)
### PYR-RESP-0347 (`COMP-026-R01`) - capture_interaction_requirements
- To capture Requirements for Interactions (e.g. set-up interactions ahead of time to enable push-to-talk without any delay due to establishing the connection).

### PYR-RESP-0348 (`COMP-026-R02`) - capture_interaction_constraints
- To capture any Constraints that may be applied to Participants' possible Interactions (e.g. constrain access to specific contact(s) or interaction types).

### PYR-RESP-0349 (`COMP-026-R03`) - determine_possible_interactions
- To determine how Participants can interact (e.g. radio, text message, or voice).

### PYR-RESP-0350 (`COMP-026-R04`) - determine_available_endpoints
- To determine the available human interaction Endpoints (e.g. phone number or radio channel).

### PYR-RESP-0351 (`COMP-026-R05`) - determine_if_solution_remains_feasible
- To determine if a planned or on-going Interaction_Solution remains feasible given current Constraints and Capability.

### PYR-RESP-0352 (`COMP-026-R06`) - identify_contacts
- To identify contacts for possible Interactions (e.g. users, user-aliases and groups of users that can interact).

### PYR-RESP-0353 (`COMP-026-R07`) - coordinate_interaction
- To setup, start and end an Interaction.

### PYR-RESP-0354 (`COMP-026-R08`) - determine_status_of_interaction
- To determine the status of an Interaction.

### PYR-RESP-0355 (`COMP-026-R09`) - determine_quality_of_interaction
- To determine the quality of an Interaction against given Measurement_Criterion/criteria.

### PYR-RESP-0356 (`COMP-026-R10`) - assess_interaction_support_capability
- To assess the Capability of the component to support human interaction taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0357 (`COMP-026-R11`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Human Interaction Capability assessment.

### PYR-RESP-0358 (`COMP-026-R12`) - predict_capability_progression
- To predict the progression of the Human Interaction Capability over time and with use.

## COMP-027 - Information Brokerage (5.4.2.27)
### PYR-RESP-0359 (`COMP-027-R01`) - capture_requirements_for_information_exchange
- To capture the information Exchange requirement (e.g. the information that is to be exchanged, between whom and when).

### PYR-RESP-0360 (`COMP-027-R02`) - capture_measurement_criteria_for_exchange
- To capture the Delivery_Characteristic required for an information exchange.

### PYR-RESP-0361 (`COMP-027-R03`) - capture_exchange_constraints
- To capture the constraints associated with an Exchange.

### PYR-RESP-0362 (`COMP-027-R04`) - determine_exchange_solution
- To determine an Exchange solution that meets the given Delivery_Characteristic requirements and Participant constraints for a Distributable_Item using available Exchange_Mechanism resources.

### PYR-RESP-0363 (`COMP-027-R05`) - determine_allowable_exchange
- To determine whether an Exchange using a specific combination of Distributable_Items, Participants and Exchange_Mechanisms is allowable.

### PYR-RESP-0364 (`COMP-027-R06`) - identify_exchange_in_progress_remains_feasible
- To identify if an Exchange in progress remains feasible, taking account of current resource constraints.

### PYR-RESP-0365 (`COMP-027-R07`) - instigate_information_configuration
- To instigate the transformation of a Distributable_Item to meet an Information_Configuration required by a Participant (including the combination of Distributable_Items from multiple sources).

### PYR-RESP-0366 (`COMP-027-R08`) - instigate_information_exchange
- To place the requirements for information Exchange onto Participants and Exchange_Mechanisms.

### PYR-RESP-0367 (`COMP-027-R09`) - determine_quality_of_exchange_delivery
- To determine the quality of an information Exchange, measured against given requirements and measurement criteria.

### PYR-RESP-0368 (`COMP-027-R10`) - capture_exchange_mechanism_of_participants
- To capture the information Exchange_Mechanisms of Participants.

### PYR-RESP-0369 (`COMP-027-R11`) - assess_information_exchange_capability
- To assess the capability to provide information Exchange taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0370 (`COMP-027-R12`) - identify_missing_information
- To identify what information is missing that could improve the certainty or specificity of the Exchange capability assessment.

### PYR-RESP-0371 (`COMP-027-R13`) - predict_capability_progression
- To predict the progression of an Exchange capability over time and with use.

## COMP-028 - Information Presentation (5.4.2.28)
### PYR-RESP-0372 (`COMP-028-R01`) - capture_conveyance_requirement
- To capture Requirements for Presentation_Interactions. This can be for information coming into the system or from the system.

### PYR-RESP-0373 (`COMP-028-R02`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is achievable given current or predicted Presentation_Capability and any Context.

### PYR-RESP-0374 (`COMP-028-R03`) - determine_presentation_solution
- To determine the method of Presentation_Interaction that meets the Requirement.

### PYR-RESP-0375 (`COMP-028-R04`) - identify_pre-conditions
- To identify Pre-conditions required to permit the conveyance of information.

### PYR-RESP-0376 (`COMP-028-R05`) - fulfil_requirement
- To convey the information coming into the system or from the system.

### PYR-RESP-0377 (`COMP-028-R06`) - capture_external_factors
- To capture Contexts that will influence the chosen mode of Presentation_Interaction.

### PYR-RESP-0378 (`COMP-028-R07`) - capture_perception_efficacy
- To capture the Perception_Efficacy for a mode of Presentation_Interaction.

### PYR-RESP-0379 (`COMP-028-R08`) - assess_information_presentation_capability
- To assess the Presentation_Capability taking account the health of resources and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0380 (`COMP-028-R09`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Presentation_Capability assessment.

### PYR-RESP-0381 (`COMP-028-R10`) - predict_capability_progression
- To predict the progression of Presentation_Capability over time and with use.

## COMP-029 - Interlocks (5.4.2.29)
### PYR-RESP-0382 (`COMP-029-R01`) - capture_function_requests
- To capture Requests for enabling or disabling a given Function_Interlock.

### PYR-RESP-0383 (`COMP-029-R02`) - determine_interlock
- To determine whether the Conditional_Check for a Function_Interlock is satisfied.

### PYR-RESP-0384 (`COMP-029-R03`) - determine_if_conditional_check_remains_feasible
- To determine if a planned or on-going Conditional_Check remains feasible.

### PYR-RESP-0385 (`COMP-029-R04`) - identify_current_states
- To maintain a view of the current states of Function_Interlocks.

### PYR-RESP-0386 (`COMP-029-R05`) - control_interlock
- To control Function_Interlocks to enable or disable functions.

### PYR-RESP-0387 (`COMP-029-R06`) - identify_interlock_request_progress
- To identify the progress of an implementation of a Conditional_Check in response to a Request to enable or disable the operation of a function.

### PYR-RESP-0388 (`COMP-029-R07`) - capture_conditional_check
- To capture the Conditional_Checks (required conditions) that define when Function_Interlocks can be enabled or disabled.

### PYR-RESP-0389 (`COMP-029-R08`) - capture_source_parameters
- To capture currently provided Source_Parameters used to determine whether Function_Interlocks should be enabled or disabled.

### PYR-RESP-0390 (`COMP-029-R09`) - assess_interlocks_capability
- To assess the Capability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

<a id="comp-030"></a>
## COMP-030 - Inventory (5.4.2.30)
### PYR-RESP-0391 (`COMP-030-R01`) - verify_inventory
- To verify the current Inventory against the planned Inventory and to verify that an Inventory is a Legal_Inventory.

### PYR-RESP-0392 (`COMP-030-R02`) - identify_whether_inventory_verification_remains_achievable
- To identify whether the requirement to determine and verify an Inventory is still achievable given current or predicted capability and conditions.

### PYR-RESP-0393 (`COMP-030-R03`) - determine_inventory
- To determine the presence and identity of physical Items within the system Inventory.

### PYR-RESP-0394 (`COMP-030-R04`) - identify_progress_of_inventory_verification
- To identify the progress of Inventory determination and verification against the requirement.

### PYR-RESP-0395 (`COMP-030-R05`) - assess_capability
- To assess Inventory Capability, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0396 (`COMP-030-R06`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Inventory Capability assessment.

### PYR-RESP-0397 (`COMP-030-R07`) - predict_inventory_verification_capability_progression
- To predict the progression of Inventory Capability over time and with use.

## COMP-031 - Jettison (5.4.2.31)
### PYR-RESP-0398 (`COMP-031-R01`) - capture_jettison_requirements
- To capture provided Requirements (e.g. mass to be removed) for jettison activities.

### PYR-RESP-0399 (`COMP-031-R02`) - capture_measurement_criteria_for_jettison_actions
- To capture provided Measurement_Criterion/criteria for jettison activities.

### PYR-RESP-0400 (`COMP-031-R03`) - capture_jettison_constraints
- To capture provided Constraints (e.g. stores that are not to be jettisoned) for jettison activities.

### PYR-RESP-0401 (`COMP-031-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current or predicted Jettison_Capability and Constraints.

### PYR-RESP-0402 (`COMP-031-R05`) - determine_jettison_solution
- To determine a Jettison_Solution that meets the given Requirements and Constraints for jettison using available Jettison_Resources.

### PYR-RESP-0403 (`COMP-031-R06`) - determine_predicted_quality_of_jettison_deliverables
- To determine the predicted quality of a proposed Jettison_Solution against given Measurement_Criterion/criteria.

### PYR-RESP-0404 (`COMP-031-R07`) - identify_pre-conditions
- To identify Pre-conditions required to support the Jettison_Solution or a step of the jettison solution.

### PYR-RESP-0405 (`COMP-031-R08`) - coordinate_jettison_dependencies
- To coordinate the execution of a Jettison_Solution.

### PYR-RESP-0406 (`COMP-031-R09`) - identify_progress_of_jettison_solution
- To identify the progress of a Jettison_Solution against the Requirements.

### PYR-RESP-0407 (`COMP-031-R10`) - determine_actual_quality_of_jettison_deliverables
- To determine the actual quality of the Jettison_Solution against the Measurement_Criterion/criteria.

### PYR-RESP-0408 (`COMP-031-R11`) - capture_jettison_locations
- To capture locations in which jettison is allowable.

### PYR-RESP-0409 (`COMP-031-R12`) - assess_jettison_capability
- To assess the Jettison_Capability to perform actions taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0410 (`COMP-031-R13`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Jettison_Capability assessment.

### PYR-RESP-0411 (`COMP-031-R14`) - predict_capability_progression
- To predict the progression of Jettison_Capability over time and with use.

## COMP-032 - Lights (5.4.2.32)
### PYR-RESP-0412 (`COMP-032-R01`) - capture_lighting_requirements
- To capture provided lighting Requirements.

### PYR-RESP-0413 (`COMP-032-R02`) - capture_lighting_constraints
- To capture Constraints on any potential Lighting_Solution.

### PYR-RESP-0414 (`COMP-032-R03`) - determine_lighting_solution
- To determine the available Lighting_Solution which best meets the Requirements and Constraints.

### PYR-RESP-0415 (`COMP-032-R04`) - identify_lighting_solution_in_progress_remains_feasible
- To identify whether a Lighting_Solution in progress remains feasible given current resources.

### PYR-RESP-0416 (`COMP-032-R05`) - implement_lighting_solution
- To control lighting in accordance with a planned Lighting_Solution.

### PYR-RESP-0417 (`COMP-032-R06`) - assess_capability
- To assess the Capability to provide lighting taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0418 (`COMP-032-R07`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0419 (`COMP-032-R08`) - predict_capability_progression
- To predict the progression of the Lights Capability over time and with use.

## COMP-033 - Location and Orientation (5.4.2.33)
### PYR-RESP-0420 (`COMP-033-R01`) - capture_parameter_requirements
- To capture the Requirements for determining Location, Orientation and Derivatives.

### PYR-RESP-0421 (`COMP-033-R02`) - capture_constraints
- To capture Constraints on the use of Sources.

### PYR-RESP-0422 (`COMP-033-R03`) - determine_if_requirement_is_achievable
- To determine if a Requirement is achievable given current Capability and Constraints.

### PYR-RESP-0423 (`COMP-033-R04`) - determine_location
- To determine the Location of a Platform relative to a Reference_Frame.

### PYR-RESP-0424 (`COMP-033-R05`) - determine_orientation
- To determine the Orientation of a Platform relative to a Reference_Frame.

### PYR-RESP-0425 (`COMP-033-R06`) - determine_derivatives
- To determine the derivatives of Location and/or Orientation relative to the Reference_Frame.

### PYR-RESP-0426 (`COMP-033-R07`) - determine_parameter_quality
- To determine the accuracy and precision of a Location, Orientation or Derivative parameter.

### PYR-RESP-0427 (`COMP-033-R08`) - capture_reference_frame
- To capture given Reference_Frames. This includes absolute and relative Reference_Frames.

### PYR-RESP-0428 (`COMP-033-R09`) - capture_source_parameters
- To capture given Source_Parameters from available Sources.

### PYR-RESP-0429 (`COMP-033-R10`) - assess_parameter_capability
- To identify the system's Capability to determine the Location and/or Orientation (or a Derivative of these parameters) of a Platform, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0430 (`COMP-033-R11`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0431 (`COMP-033-R12`) - predict_capability_progression
- To predict the progression of the Location and Orientation Capability over time and with use.

## COMP-034 - Mass and Balance (5.4.2.34)
### PYR-RESP-0432 (`COMP-034-R01`) - capture_inertia_and_balance_requirements
- To capture requirements for maintaining the Moment_of_Inertia and Centre_of_Mass for a given Configuration.

### PYR-RESP-0433 (`COMP-034-R02`) - capture_mass_and_balance_limit
- To capture the Mass_and_Balance_Limits.

### PYR-RESP-0434 (`COMP-034-R03`) - determine_changes_conform_to_limits
- To determine whether Configuration changes (e.g. a planned stores release package) conform to Mass_and_Balance_Limits.

### PYR-RESP-0435 (`COMP-034-R04`) - determine_total_mass
- To determine the Total_Mass of Contributing_Elements within Configurations.

### PYR-RESP-0436 (`COMP-034-R05`) - determine_centre_of_mass
- To determine the Centre_of_Mass of Contributing_Elements within Configurations.

### PYR-RESP-0437 (`COMP-034-R06`) - determine_moment_of_inertia
- To determine the Moment_of_Inertia of Contributing_Elements within Configurations.

### PYR-RESP-0438 (`COMP-034-R07`) - determine_mass_changes
- To determine Total_Mass, Moment_of_Inertia and Centre_of_Mass changes which would result from variations in the Contributing_Element(s) within Configurations.

### PYR-RESP-0439 (`COMP-034-R08`) - determine_optimum_configuration_balance
- To determine the optimum Configuration of Contributing_Elements taking into account Total_Mass, Moment_of_Inertia and Centre_of_Mass in order to maintain balance of the Exploiting Platform.

### PYR-RESP-0440 (`COMP-034-R09`) - suggest_mass_reconfiguration
- To determine Contributing_Element(s) which could be varied in order to achieve the desired Total_Mass, Moment_of_Inertia or Centre_of_Mass of a Configuration.

### PYR-RESP-0441 (`COMP-034-R10`) - capture_current_mass_and_balance_configs
- To capture current Configurations of Contributing_Elements.

### PYR-RESP-0442 (`COMP-034-R11`) - capture_planned_mass_and_balance_configs
- To capture planned Configurations of Contributing_Elements.

### PYR-RESP-0443 (`COMP-034-R12`) - assess_mass_and_balance_capability
- To predict the progression of the capability to determine whether potential Configuration changes will conform to mass and balance limits and to determine Configuration changes required to maintain mass and balance limits or optimise the Configuration, over time and with use.

### PYR-RESP-0444 (`COMP-034-R13`) - predict_capability_progression
- To predict the progression of available Capability over time and with use.

### PYR-RESP-0445 (`COMP-034-R14`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the capability assessment.

## COMP-035 - Mechanical Positioning (5.4.2.35)
### PYR-RESP-0446 (`COMP-035-R01`) - capture_positioning_requirements
- To capture provided positioning Requirements for a Physical_Element.

### PYR-RESP-0447 (`COMP-035-R02`) - capture_measurement_criteria
- To capture given Measurement_Criterion.

### PYR-RESP-0448 (`COMP-035-R03`) - capture_positioning_constraints
- To capture provided Physical_Element_Constraints for a Physical_Element.

### PYR-RESP-0449 (`COMP-035-R04`) - capture_effector_constraints
- To capture provided Effector_Constraints for an Effector.

### PYR-RESP-0450 (`COMP-035-R05`) - determine_required_position_solution
- To determine the Effector positions to achieve the required position of a Physical_Element.

### PYR-RESP-0451 (`COMP-035-R06`) - identify_positioning_solution_in_progress_remains_feasible
- To identify whether a Positioning_Solution in progress remains feasible against particular Requirements and Measurement_Criterion/criteria given current resources.

### PYR-RESP-0452 (`COMP-035-R07`) - control_position
- To control the position of a Physical_Element by placing positioning dependencies on Effectors.

### PYR-RESP-0453 (`COMP-035-R08`) - identify_positioning_progress
- To identify the progress of a Positioning_Solution against a Requirement.

### PYR-RESP-0454 (`COMP-035-R09`) - assess_movement_capability
- To assess the Movement_Capability of a Physical_Element taking account of the capability of the Effector(s), system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0455 (`COMP-035-R10`) - identify_missing_capability_information
- To identify missing information which could improve the certainty or specificity of Movement_Capability determination.

### PYR-RESP-0456 (`COMP-035-R11`) - predict_movement_capability_progression
- To predict the progression of the Movement_Capability of a Physical_Element over time and with use.

## COMP-036 - Navigation Sensing (5.4.2.36)
### PYR-RESP-0457 (`COMP-036-R01`) - capture_navigation_requirements
- To capture the Requirements for navigation, e.g. provision of the most accurate or most robust positional information.

### PYR-RESP-0458 (`COMP-036-R02`) - capture_solution_measurement_criteria
- To capture given Measurement_Criterion/criteria (e.g. stability or timing) against which a Navigation_Solution_Scheme will be assessed.

### PYR-RESP-0459 (`COMP-036-R03`) - capture_navigation_solution_constraints
- To capture the Constraints which must be observed when determining a Navigation_Solution_Scheme (e.g. resource restrictions, transmission restrictions, or exclusions).

### PYR-RESP-0460 (`COMP-036-R04`) - identify_whether_requirement_is_achievable
- To identify whether a Requirement is achievable given the current or predicted Capability and Constraints.

### PYR-RESP-0461 (`COMP-036-R05`) - determine_navigation_solution
- To determine a Navigation_Solution_Scheme using the available Capability, which meets the Requirements and satisfies the Constraints.

### PYR-RESP-0462 (`COMP-036-R06`) - determine_predicted_quality_of_navigation_solution_scheme
- To determine the predicted quality of a Navigation_Solution_Scheme against the Measurement_Criterion/criteria.

### PYR-RESP-0463 (`COMP-036-R07`) - determine_solution_actions
- To determine the activities required to support a Navigation_Solution_Scheme (e.g. the configuration of Navigation_Resources, for the provision or processing of navigation data, and determination of required support activities).

### PYR-RESP-0464 (`COMP-036-R08`) - determine_solution_dependencies
- To identify dependencies to support a Navigation_Solution_Scheme or a step of the Navigation_Solution_Scheme.

### PYR-RESP-0465 (`COMP-036-R09`) - coordinate_navigation_solution
- To coordinate the actions required to implement a Navigation_Solution_Scheme, e.g. by commanding the instruction of a Navigation_Resource for the provision or processing of navigation data.

### PYR-RESP-0466 (`COMP-036-R10`) - identify_progress_of_navigation_solution
- To identify the progress of a Navigation_Solution_Scheme and achievement against the Requirement.

### PYR-RESP-0467 (`COMP-036-R11`) - determine_actual_quality_of_navigation_solution_scheme
- To determine the actual quality of the delivered Navigation_Solution_Scheme, measured against the Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0468 (`COMP-036-R12`) - determine_solution_cost
- To determine the cost of a Navigation_Solution_Scheme against given Measurement_Criterion/criteria.

### PYR-RESP-0469 (`COMP-036-R13`) - assess_capability
- To assess the Capability of the component taking account of capability of Navigation_Resources, system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0470 (`COMP-036-R14`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of Capability determination.

### PYR-RESP-0471 (`COMP-036-R15`) - predict_capability_progression
- To predict the progression of Capability over time and with use.

## COMP-037 - Network Routes (5.4.2.37)
### PYR-RESP-0472 (`COMP-037-R01`) - capture_requirements_for_network_routes
- To capture provided Transmission_Requirements.

### PYR-RESP-0473 (`COMP-037-R02`) - capture_measurement_criteria_for_network_routes
- To capture provided Measurement_Criterion/criteria.

### PYR-RESP-0474 (`COMP-037-R03`) - capture_constraints_for_network_routes
- To capture provided Constraints for the use of network routes.

### PYR-RESP-0475 (`COMP-037-R04`) - manage_network_route_congestion
- To determine and manage the level of congestion within network Nodes.

### PYR-RESP-0476 (`COMP-037-R05`) - determine_next_hop
- To select a Next_Hop from the Transmission_Routes.

### PYR-RESP-0477 (`COMP-037-R06`) - determine_solution_feasibility
- To determine if a planned or on-going Next_Hop remains feasible given current capability and Constraints.

### PYR-RESP-0478 (`COMP-037-R07`) - maintain_network_configuration
- To maintain the configuration of the network.

### PYR-RESP-0479 (`COMP-037-R08`) - convey_data
- To convey data over a Next_Hop.

### PYR-RESP-0480 (`COMP-037-R09`) - determine_network_route_quality
- To determine the Transmission_Quality - including but not limited to bandwidth utilisation, delay over a route and packet error rate.

### PYR-RESP-0481 (`COMP-037-R10`) - assess_network_routes_capability
- To assess the capability provided by the network resources taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0482 (`COMP-037-R11`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the capability assessment.

### PYR-RESP-0483 (`COMP-037-R12`) - predict_network_routes_capability_progression
- To predict the progression of the capability of network routes over time and with use.

## COMP-038 - Networks (5.4.2.38)
### PYR-RESP-0484 (`COMP-038-R01`) - capture_network_requirements
- To capture given network Connectivity_Requirements (e.g. the need to establish a new Connection).

### PYR-RESP-0485 (`COMP-038-R02`) - capture_network_measurement_criteria
- To capture given Performance/criteria (e.g. bandwidth, latency, or reach) for networks.

### PYR-RESP-0486 (`COMP-038-R03`) - capture_network_constraints
- To capture given network Constraints (e.g. integrity, security, or safety).

### PYR-RESP-0487 (`COMP-038-R04`) - determine_network_solution
- Description not extracted from source text.

### PYR-RESP-0488 (`COMP-038-R05`) - identify_network_solution_in_progress_remains_feasible
- To identify whether a network Connection in progress remains feasible against particular Connectivity_Requirements and Performance/criteria given current resources.

### PYR-RESP-0489 (`COMP-038-R06`) - determine_network_performance
- To determine the load and performance of a network in terms of the availability and usage of Network_Resources.

### PYR-RESP-0490 (`COMP-038-R07`) - identify_network_pre-conditions
- To identify Pre-Conditions required for a Connection.

### PYR-RESP-0491 (`COMP-038-R08`) - identify_network_change
- To identify divergence from the expected Topology or network performance.

### PYR-RESP-0492 (`COMP-038-R09`) - maintain_network
- To establish and maintain a network Topology by management of Network_Resources.

### PYR-RESP-0493 (`COMP-038-R10`) - determine_network_solution_quality
- To determine the quality of a Connection.

### PYR-RESP-0494 (`COMP-038-R11`) - determine_network_capability
- To determine the Capability to provide networks using available Network_Resources, taking into account observed anomalies.

### PYR-RESP-0495 (`COMP-038-R12`) - identify_missing_capability_information
- To identify missing information which could improve the certainty or specificity of network Capability determination.

### PYR-RESP-0496 (`COMP-038-R13`) - predict_network_capability
- To predict the progression of network Capability over time and with use.

## COMP-039 - Objectives (5.4.2.39)
### PYR-RESP-0497 (`COMP-039-R01`) - capture_requirements
- To capture Objectives (e.g. objective type, timing and risk profile).

### PYR-RESP-0498 (`COMP-039-R02`) - capture_measurement_criteria
- To capture given quality requirements for Objective_Solutions (e.g. quality, risk and robustness).

### PYR-RESP-0499 (`COMP-039-R03`) - capture_constraints
- To capture Constraints on how an Objective may be achieved (e.g. operator imposed restrictions or Rules of engagement).

### PYR-RESP-0500 (`COMP-039-R04`) - identify_whether_requirement_remains_achievable
- To identify whether an Objective is still achievable given current Flight_Capability and Constraints.

### PYR-RESP-0501 (`COMP-039-R05`) - determine_implementation_scheme
- To determine an Objective_Solution to achieve an Objective.

### PYR-RESP-0502 (`COMP-039-R06`) - determine_predicted_quality_of_solution
- To evaluate the predicted quality of a proposed Objective_Solution against given quality requirements.

### PYR-RESP-0503 (`COMP-039-R07`) - identify_dependencies
- To identify the interdependencies between Tasks required to support the delivery of an Objective_Solution.

### PYR-RESP-0504 (`COMP-039-R08`) - satisfy_dependencies_between_tasks
- To satisfy the interdependencies between Tasks through the management of Tasks.

### PYR-RESP-0505 (`COMP-039-R09`) - coordinate_objective_enactment
- To coordinate the enactment of an Objective_Solution via the fulfilment of a set of coordinated Tasks.

### PYR-RESP-0506 (`COMP-039-R10`) - identify_progress_of_objective
- To identify the progress of an Objective's Objective_Solution.

### PYR-RESP-0507 (`COMP-039-R11`) - determine_actual_quality_of_solution
- To evaluate the quality of the delivered Objective_Solution against given quality requirements.

### PYR-RESP-0508 (`COMP-039-R12`) - evaluate_implementation_scheme_cost
- To evaluate the costs of a planned Objective_Solution against given measurement criteria.

### PYR-RESP-0509 (`COMP-039-R13`) - determine_capability
- To determine Flight_Capability based on Participant_Capability, taking into account observed anomalies.

### PYR-RESP-0510 (`COMP-039-R14`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the capability assessment.

### PYR-RESP-0511 (`COMP-039-R15`) - predict_capability_progression
- To predict the progression of available Flight_Capability over time and with use.

<a id="comp-040"></a>
## COMP-040 - Observability (5.4.2.40)
### PYR-RESP-0512 (`COMP-040-R01`) - capture_observability_requirements
- To capture requirements for determining the Observability of a Subject.

### PYR-RESP-0513 (`COMP-040-R02`) - capture_observability_measurement_criteria
- To capture Measurement_Criterion/criteria for Observability.

### PYR-RESP-0514 (`COMP-040-R03`) - capture_observability_constraints
- To capture Constraints on Observability (e.g. Observation_Means not to be used, effect of range and occlusion on Observation_Means).

### PYR-RESP-0515 (`COMP-040-R04`) - determine_observability
- To determine whether an Observer can observe a Subject taking into account Observation_Means, Observability_Obstacles and any Constraints.

### PYR-RESP-0516 (`COMP-040-R05`) - determine_observability_threshold
- To determine the Observability_Threshold, for a given Observable_Property and with any Observability_Obstacles, at which a Subject will become observable to a given Observer.

### PYR-RESP-0517 (`COMP-040-R06`) - determine_line_of_sight
- To determine Observation_Path between an Observer and a Subject.

### PYR-RESP-0518 (`COMP-040-R07`) - determine_quality_of_observability_service
- To determine the quality of Observability against given Measurement_Criterion/criteria.

### PYR-RESP-0519 (`COMP-040-R08`) - assess_observability_service_capability
- To assess the Capability to determine Observability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0520 (`COMP-040-R09`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0521 (`COMP-040-R10`) - predict_observability_capability_progression
- To predict the progression of the Observability Capability over time and with use.

## COMP-041 - Operational Rules and Limits (5.4.2.41)
### PYR-RESP-0522 (`COMP-041-R01`) - capture_rules
- To capture Operational_Rules.

### PYR-RESP-0523 (`COMP-041-R02`) - determine_applicable_rules
- To determine which Operational_Rules are applicable under given conditions related to current or forecast Conditions.

### PYR-RESP-0524 (`COMP-041-R03`) - determine_applicable_limits
- To determine which Limits are applicable under applicable rules.

### PYR-RESP-0525 (`COMP-041-R04`) - determine_breach_of_rule
- To determine which Operational_Rules are broken following a Limit_Breach.

### PYR-RESP-0526 (`COMP-041-R05`) - assess_capability
- To assess the Capability to provide Limits and report Rule_Breaches.

### PYR-RESP-0527 (`COMP-041-R06`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0528 (`COMP-041-R07`) - predict_capability_progression
- To predict the progression of Capability over time and with use.

## COMP-042 - Pointing (5.4.2.42)
### PYR-RESP-0529 (`COMP-042-R01`) - capture_orientation_requirements
- To capture the orientation Requirements (e.g. required position or turn rate).

### PYR-RESP-0530 (`COMP-042-R02`) - capture_orientation_measurement_criteria
- To capture provided criteria that an Orientation_Solution will be measured against.

### PYR-RESP-0531 (`COMP-042-R03`) - capture_orientation_constraints
- To capture provided Constraints for Orientation actions.

### PYR-RESP-0532 (`COMP-042-R04`) - determine_orientation_solution
- To determine an Orientation_Solution that meets the given Requirements and Constraints.

### PYR-RESP-0533 (`COMP-042-R05`) - determine_current_orientation
- To determine the current Orientation of a Controllable_Element.

### PYR-RESP-0534 (`COMP-042-R06`) - identify_orientation_solution_in_progress_remains_feasible
- To identify whether an Orientation_Solution in progress remains feasible.

### PYR-RESP-0535 (`COMP-042-R07`) - coordinate_orientation_solution
- To coordinate the Orientation_Solution to ensure the individual Controllable_Element is oriented correctly (either mechanically or electronically).

### PYR-RESP-0536 (`COMP-042-R08`) - identify_progress_of_orientation_solution
- To identify the progress of an Orientation_Solution against the Requirements.

### PYR-RESP-0537 (`COMP-042-R09`) - determine_quality_of_orientation_solution
- To determine the quality of a proposed Orientation_Solution against given Measurement_Criterion/criteria.

### PYR-RESP-0538 (`COMP-042-R10`) - determine_quality_of_deliverables
- To determine the quality of the outcomes generated by executing an Orientation_Solution, measured against given Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0539 (`COMP-042-R11`) - assess_orientation_capability
- To assess the Orientation_Capability available to the Pointing component taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0540 (`COMP-042-R12`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Orientation_Capability assessment.

### PYR-RESP-0541 (`COMP-042-R13`) - predict_capability_progression
- To predict the progression of the Orientation_Capability over time and with use.

## COMP-043 - Power (5.4.2.43)
### PYR-RESP-0542 (`COMP-043-R01`) - capture_power_solution_requirements
- To capture Power_Requirements for Power_Solutions.

### PYR-RESP-0543 (`COMP-043-R02`) - capture_measurement_criteria_for_power_solution
- To capture provided Measurement_Criterion/criteria for power solutions.

### PYR-RESP-0544 (`COMP-043-R03`) - capture_power_solution_constraints
- To capture provided Equipment_Constraints for use of power, such as the restriction of the use of a specific Power_Source.

### PYR-RESP-0545 (`COMP-043-R04`) - identify_whether_power_requirement_remains_achievable
- To identify if a Power_Requirement is still achievable given current resources.

### PYR-RESP-0546 (`COMP-043-R05`) - determine_power_solution
- To determine the appropriate Power_Solution to meet captured Power_Requirements and Equipment_Constraints.

### PYR-RESP-0547 (`COMP-043-R06`) - determine_quality_of_designed_solution
- To determine the Designed_Quality of the Power_Solution against one or more given Measurement_Criterion.

### PYR-RESP-0548 (`COMP-043-R07`) - coordinate_power_resources
- To coordinate the use of Power_Sources, Power_Sinks and Power_Regulators to meet the requirements of a Power_Solution.

### PYR-RESP-0549 (`COMP-043-R08`) - identify_progress_of_power_solution
- To identify the progress of a Power_Solution against the Power_Requirements.

### PYR-RESP-0550 (`COMP-043-R09`) - determine_quality_of_delivered_solution
- To determine the Delivered_Quality of the delivered Power_Solution measured against Power_Requirement in terms of given Measurement_Criterion.

### PYR-RESP-0551 (`COMP-043-R10`) - assess_power_capability
- To assess the current capability to manage Power_Sources, Power_Sinks and Power_Regulators.

### PYR-RESP-0552 (`COMP-043-R11`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the power capability assessment.

### PYR-RESP-0553 (`COMP-043-R12`) - predict_power_capability_progression
- To predict the progression of power capability over time and with use.

## COMP-044 - Propulsion (5.4.2.44)
### PYR-RESP-0554 (`COMP-044-R01`) - capture_propulsion_requirements
- To capture Thrust_Requirements.

### PYR-RESP-0555 (`COMP-044-R02`) - capture_state_requirements
- To capture State_Requirements.

### PYR-RESP-0556 (`COMP-044-R03`) - capture_measurement_criteria_for_propulsion_solution
- To capture provided Measurement_Criterion for propulsion solutions.

### PYR-RESP-0557 (`COMP-044-R04`) - capture_propulsion_constraints
- To capture provided Constraints for use of propulsion resources.

### PYR-RESP-0558 (`COMP-044-R05`) - identify_whether_thrust_requirement_remains_achievable
- To identify if a Thrust_Requirement is still achievable given current resources.

### PYR-RESP-0559 (`COMP-044-R06`) - identify_whether_state_requirement_remains_achievable
- To identify if a State_Requirement is still achievable given current resources.

### PYR-RESP-0560 (`COMP-044-R07`) - control_propulsion
- To control the Propulsion_Unit_Setting(s) for the output required.

### PYR-RESP-0561 (`COMP-044-R08`) - identify_progress_of_delivered_thrust
- To identify the progress of Thrust against the Thrust_Requirements.

### PYR-RESP-0562 (`COMP-044-R09`) - determine_quality_of_propulsion_unit_setting
- To determine the Designed_Thrust_Quality of the Propulsion_Unit_Setting against one or more given Measurement_Criterion.

### PYR-RESP-0563 (`COMP-044-R10`) - determine_quality_of_delivered_thrust
- To determine the Delivered_Thrust_Quality of the Thrust measured against Thrust_Requirement in terms of given Measurement_Criterion.

### PYR-RESP-0564 (`COMP-044-R11`) - assess_capability
- To assess Capability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0565 (`COMP-044-R12`) - identify_missing_propulsion_information
- To identify missing information which could improve the certainty or specificity of Capability assessment.

### PYR-RESP-0566 (`COMP-044-R13`) - predict_propulsion_capability
- To predict the progression of Capability over time and with use (e.g. in the presence of degrading failures).

## COMP-045 - Reference Times (5.4.2.45)
### PYR-RESP-0567 (`COMP-045-R01`) - determine_difference_between_reference_times
- To determine the Difference between Reference_Times.

### PYR-RESP-0568 (`COMP-045-R02`) - determine_accuracy_of_a_time
- To determine the Accuracy of a Reference_Time.

### PYR-RESP-0569 (`COMP-045-R03`) - determine_confidence_in_a_time
- To determine the reported Confidence in a Reference_Time, relative to one or more of the declared Accuracy or the declared Precision.

### PYR-RESP-0570 (`COMP-045-R04`) - determine_choice_of_time
- To determine the Reference_Time for a specific Use.

### PYR-RESP-0571 (`COMP-045-R05`) - capture_the_precision_of_time
- To capture the Precision of a Reference_Time.

### PYR-RESP-0572 (`COMP-045-R06`) - assess_capability
- To assess the capability to represent Reference_Times, their Quality, and their Use.

### PYR-RESP-0573 (`COMP-045-R07`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0574 (`COMP-045-R08`) - predict_capability_progression
- To predict the progression of Reference Times Capability over time.

## COMP-046 - Release Aiming (5.4.2.46)
### PYR-RESP-0575 (`COMP-046-R01`) - capture_release_requirements
- To capture the Requirements to be fulfilled by the Aiming_Solution.

### PYR-RESP-0576 (`COMP-046-R02`) - capture_measurement_criteria
- To capture provided Measurement_Criterion/criteria for Aiming_Solutions.

### PYR-RESP-0577 (`COMP-046-R03`) - capture_release_constraints
- To capture the externally imposed Constraints that limit where or how the store can be released.

### PYR-RESP-0578 (`COMP-046-R04`) - determine_aiming_solutions
- To determine Aiming_Solutions.

### PYR-RESP-0579 (`COMP-046-R05`) - determine_predicted_quality_of_aiming_solution
- To determine the predicted quality of the Aiming_Solution against provided Measurement_Criterion/criteria.

### PYR-RESP-0580 (`COMP-046-R06`) - identify_aiming_solution_in_progress_remains_feasible
- To identify whether an Aiming_Solution in progress remains feasible given current resources.

### PYR-RESP-0581 (`COMP-046-R07`) - assess_capability
- To assess the Capability to provide Release Aiming's services taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0582 (`COMP-046-R08`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0583 (`COMP-046-R09`) - predict_aiming_capability_progression
- To predict the progression of Release Aiming's aiming Capability over time and with use.

## COMP-047 - Release Effecting (5.4.2.47)
### PYR-RESP-0584 (`COMP-047-R01`) - capture_store_release_requirements
- To capture provided Requirements for a Store release (e.g. release Store 'X' from Location 'Y' in an armed state).

### PYR-RESP-0585 (`COMP-047-R02`) - capture_release_effecting_constraints
- To capture provided Constraints, e.g. Stores are not to be released for training activities.

### PYR-RESP-0586 (`COMP-047-R03`) - identify_whether_release_requirement_is_achievable
- To identify whether a Store release Requirement is achievable given current Release_Effecting_Capability.

### PYR-RESP-0587 (`COMP-047-R04`) - determine_release_effecting_steps
- To determine the Release_Timeline and the associated Release_Effecting_Steps that meet the given Requirements and Constraints for the release of a Store using available Release_Effecting_Resources.

### PYR-RESP-0588 (`COMP-047-R05`) - determine_release_element_states
- To determine the current state of release information on Stores and Release_Effecting_Resources.

### PYR-RESP-0589 (`COMP-047-R06`) - identify_pre-conditions
- To identify Pre-conditions required to support Release_Effecting_Steps that fulfil the Release_Timeline.

### PYR-RESP-0590 (`COMP-047-R07`) - coordinate_use_of_release_effecting_resources
- To coordinate Release_Effecting_Resources to effect an operational release or jettison by implementing Release_Effecting_Steps in accordance with a Release_Timeline.

### PYR-RESP-0591 (`COMP-047-R08`) - determine_store_release_progress
- To determine the progress of the Store release (e.g. report Store release in progress, Store released, Store jettisoned, Store misfired, or Store hung).

### PYR-RESP-0592 (`COMP-047-R09`) - assess_release_effecting_capability
- To assess the Release_Effecting_Capability to effect an operational release or jettison, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0593 (`COMP-047-R10`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Release_Effecting_Capability assessment.

### PYR-RESP-0594 (`COMP-047-R11`) - predict_capability_progression
- To predict the progression of the Release_Effecting_Capability over time and with use.

## COMP-048 - Routes (5.4.2.48)
### PYR-RESP-0595 (`COMP-048-R01`) - capture_positioning_requirements
- To capture given Positioning_Requirements.

### PYR-RESP-0596 (`COMP-048-R02`) - capture_measurement_criteria
- To capture provided Measurement_Criterion (e.g. fuel Cost) for Routes.

### PYR-RESP-0597 (`COMP-048-R03`) - capture_routing_constraints
- To capture given Routing_Constraints (e.g. weather volumes and threat volumes).

### PYR-RESP-0598 (`COMP-048-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Positioning_Requirement is still achievable given current or predicted Routing_Capability.

### PYR-RESP-0599 (`COMP-048-R05`) - determine_route
- To determine a Route that meets the given Positioning_Requirements within the Vehicle_Capability and the given Routing_Constraint (e.g. volumes to remain within or avoid).

### PYR-RESP-0600 (`COMP-048-R06`) - identify_pre-conditions
- To identify Pre-conditions required to support a Route or a portion of a Route.

### PYR-RESP-0601 (`COMP-048-R07`) - command_route
- To execute a selected Route by commanding the Vehicle to follow a sequence of routepoints.

### PYR-RESP-0602 (`COMP-048-R08`) - determine_route_progress
- To determine the progress of a Vehicle against the selected Route.

### PYR-RESP-0603 (`COMP-048-R09`) - determine_routing_continuity
- To determine the positional continuity between the adjacent elements of a planned path as well as the overall completeness of the planned path. This could be as part of pre-planning, or ensuring continuity between what is currently being enacted and subsequently planned.

### PYR-RESP-0604 (`COMP-048-R10`) - collate_route_cost
- To collate the Cost (e.g. time or fuel use) for any generated Route against the provided Measurement_Criterion.

### PYR-RESP-0605 (`COMP-048-R11`) - assess_routing_capability
- To determine the Routing_Capability, taking into account system health and observed anomalies.

### PYR-RESP-0606 (`COMP-048-R12`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Routing_Capability assessment.

### PYR-RESP-0607 (`COMP-048-R13`) - predict_routing_capability
- To predict the progression of Routing_Capability over time and with use, taking account of system health and observed anomalies.

## COMP-049 - Semantic Translation (5.4.2.49)
### PYR-RESP-0608 (`COMP-049-R01`) - capture_interaction_requirements
- To capture provided requirements for Interactions between Systems.

### PYR-RESP-0609 (`COMP-049-R02`) - capture_interaction_constraints
- To capture provided constraints on Interactions and application of Semantic_Rules.

### PYR-RESP-0610 (`COMP-049-R03`) - determine_if_interaction_remains_achievable
- To determine if an Interaction requirement remains achievable given current Capability and Constraints.

### PYR-RESP-0611 (`COMP-049-R04`) - determine_transaction
- To determine how to meet the given requirements for an Interaction between Systems.

### PYR-RESP-0612 (`COMP-049-R05`) - deliver_system_interactions
- To apply the Semantic_Rule provided by an external System in order to translate between internal and external understandings.

### PYR-RESP-0613 (`COMP-049-R06`) - determine_quality_of_interaction
- To determine the quality of the Interaction provided by Semantic Translation during execution, measured against given requirements.

### PYR-RESP-0614 (`COMP-049-R07`) - assess_capability
- To assess the Capability to provide Semantic Translation's services taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0615 (`COMP-049-R08`) - predict_capability_progression
- To predict the progression of Semantic Translation Capability over time and with use.

<a id="comp-050"></a>
## COMP-050 - Sensing (5.4.2.50)
### PYR-RESP-0616 (`COMP-050-R01`) - capture_sensing_action_requirements
- To capture given Sensing_Requirements (e.g. target criteria, location and time).

### PYR-RESP-0617 (`COMP-050-R02`) - capture_sensing_action_measurement_criteria
- To capture given Measurement_Criterion/criteria for sensing output (e.g. recon image quality).

### PYR-RESP-0618 (`COMP-050-R03`) - capture_sensing_action_constraints
- To capture given sensing Constraints (e.g. spatial restrictions or EMCON restrictions on active sensing).

### PYR-RESP-0619 (`COMP-050-R04`) - identify_whether_requirement_remains_achievable
- To identify if a Sensing_Solution in progress remains achievable given current resources.

### PYR-RESP-0620 (`COMP-050-R05`) - determine_sensing_solution
- To determine a Sensing_Solution (i.e. a sequence of sensing activities) that meets the given Sensing_Requirements using available Sensing_Resources, within the provided Constraints and prevailing external factors (e.g. weather).

### PYR-RESP-0621 (`COMP-050-R06`) - determine_predicted_quality_of_sensing_solution
- To determine the predicted quality of a Sensing_Solution against given Measurement_Criterion/Criteria.

### PYR-RESP-0622 (`COMP-050-R07`) - identify_sensing_solution_pre-conditions
- To identify Pre-conditions to support a Sensing_Solution.

### PYR-RESP-0623 (`COMP-050-R08`) - coordinate_sensing_solution
- To execute a Sensing_Solution by commanding Sensing_Resources.

### PYR-RESP-0624 (`COMP-050-R09`) - identify_progress_of_sensing_solution
- To identify the progress of a Sensing_Solution against the Sensing_Requirement.

### PYR-RESP-0625 (`COMP-050-R10`) - capture_actual_quality_of_deliverables
- To capture the actual quality of the deliverables provided by a Sensing_Solution, measured against given Sensing_Requirements and Measurement_Criterion/Criteria.

### PYR-RESP-0626 (`COMP-050-R11`) - assess_sensing_action_capability
- To determine the available Sensing_Capability provided by installed sensing resources, taking into account anomalies and sensor health.

### PYR-RESP-0627 (`COMP-050-R12`) - identify_required_capability_information
- To identify missing resource information that is required for assessing Sensing_Capability.

### PYR-RESP-0628 (`COMP-050-R13`) - predict_sensing_action_capability_progression
- To predict the progression of Sensing_Capability over time and with use.

## COMP-051 - Sensor Data Interpretation (5.4.2.51)
### PYR-RESP-0629 (`COMP-051-R01`) - capture_requirements
- To capture provided Requirements for Interpreted_Data_Products (e.g. determine the position of objects of a particular type within a region).

### PYR-RESP-0630 (`COMP-051-R02`) - capture_measurement_criteria
- To capture the method for measuring a Data_Interpretation_Solution (e.g. by comparison with a required confidence in the result).

### PYR-RESP-0631 (`COMP-051-R03`) - identify_if_requirement_remains_achievable
- To identify if a Requirement remains achievable given current Interpretation_Resources.

### PYR-RESP-0632 (`COMP-051-R04`) - determine_solution
- To determine a coordinated and combined sequence of activities to use as the Data_Interpretation_Solution which satisfies an interpretation Requirement.

### PYR-RESP-0633 (`COMP-051-R05`) - determine_predicted_quality_of_solution
- To determine the quality of a proposed Data_Interpretation_Solution against given Measurement_Criterion.

### PYR-RESP-0634 (`COMP-051-R06`) - determine_solution_dependencies
- To identify any Sensor_Data_Provision_Dependency and Data_Interpretation_Dependency required to support the Data_Interpretation_Solution.

### PYR-RESP-0635 (`COMP-051-R07`) - coordinate_solution
- To coordinate the execution of a Data_Interpretation_Solution.

### PYR-RESP-0636 (`COMP-051-R08`) - identify_progress_of_solution
- To identify the progress of a Data_Interpretation_Solution against the Requirements.

### PYR-RESP-0637 (`COMP-051-R09`) - determine_quality_of_deliverables
- To determine the quality of the Interpreted_Data_Product, measured against given Requirements and Measurement_Criterion.

### PYR-RESP-0638 (`COMP-051-R10`) - assess_capability
- To assess the Interpretation_Resource_Capability that can be applied to sensor data, taking account of observed anomalies (e.g. erroneous outputs of activities which satisfy a Data_Interpretation_Dependency or dependencies).

### PYR-RESP-0639 (`COMP-051-R11`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Interpretation_Capability assessment.

### PYR-RESP-0640 (`COMP-051-R12`) - predict_capability_progression
- To predict the progression of Interpretation_Capability over time and with use.

## COMP-052 - Sensor Products (5.4.2.52)
### PYR-RESP-0641 (`COMP-052-R01`) - capture_requirements
- To capture Requirements to provide, manipulate or analyse Sensor_Products or to identify and characterise Features within Sensor_Products (e.g. identify and characterise shapes matching a specified pattern from imagery captured in a particular region, or identify and characterise a specific waveform pattern from an RF source).

### PYR-RESP-0642 (`COMP-052-R02`) - capture_measurement_criteria
- To capture Measurement_Criterion for Sensor_Product provision, manipulation, or analysis (e.g. Quality).

### PYR-RESP-0643 (`COMP-052-R03`) - capture_constraints
- To capture Constraints on Sensor_Product provision, manipulation, or analysis (e.g. restriction on measurement source).

### PYR-RESP-0644 (`COMP-052-R04`) - determine_if_requirement_is_achievable
- To determine if a Sensor_Product requirement is achievable given current Capability and Constraints.

### PYR-RESP-0645 (`COMP-052-R05`) - determine_solution
- To determine a solution which meets the Requirements and satisfies the Constraints for Sensor_Product provision, manipulation, or analysis.

### PYR-RESP-0646 (`COMP-052-R06`) - capture_acquisition_characteristics
- To capture the acquisition characteristics of Sensor_Products (e.g. time of capture, spatial region captured, or spectral frequency captured).

### PYR-RESP-0647 (`COMP-052-R07`) - execute_solution
- To provide, manipulate, or analyse Sensor_Products based on determined solutions.

### PYR-RESP-0648 (`COMP-052-R08`) - maintain_traceability
- To maintain the lineage of Sensor_Products, Sensor_Product_Characterisations of Sensor_Products and the Feature_Characterisation of Features identified within Sensor_Products, including Traceability to measurement sources (e.g. an external system or local sensor) and precursor Sensor_Products.

### PYR-RESP-0649 (`COMP-052-R09`) - enhance_sensor_products
- To enhance Sensor_Products in order to improve Quality (e.g. noise reduction or contrast enhancement).

### PYR-RESP-0650 (`COMP-052-R10`) - combine_sensor_products
- To combine Sensor_Products from different sources or with different acquisition characteristics, e.g. pixel level combining of imagery.

### PYR-RESP-0651 (`COMP-052-R11`) - characterise_sensor_products
- To characterise the nature of a Sensor_Product or Feature identified within a Sensor_Product (e.g. the Sensor_Product metadata, identified pattern, or confidence level).

### PYR-RESP-0652 (`COMP-052-R12`) - determine_solution_quality
- To determine the Quality of a solution against the Measurement_Criterion.

### PYR-RESP-0653 (`COMP-052-R13`) - determine_quality_of_outputs
- To determine the Quality of Sensor_Products, the Sensor_Product_Characterisation of Sensor_Products and the Feature_Characterisation of identified Features within Sensor_Products (e.g. the level of confidence associated with a specified pattern match).

### PYR-RESP-0654 (`COMP-052-R14`) - capture_sensor_products
- To capture Sensor_Products.

### PYR-RESP-0655 (`COMP-052-R15`) - assess_capability
- To assess the Capability to provide, manipulate, or analyse Sensor_Products, taking account of observed anomalies (e.g. measurement source availability).

### PYR-RESP-0656 (`COMP-052-R16`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0657 (`COMP-052-R17`) - predict_capability_progression
- To predict the progression of Sensor Products Capability over time and with use.

## COMP-053 - Sensors (5.4.2.53)
### PYR-RESP-0658 (`COMP-053-R01`) - capture_requirements_for_sensor_resources
- To capture provided requirements (e.g. turn on/off or obtain a measurement) for use of Sensor_Resources.

### PYR-RESP-0659 (`COMP-053-R02`) - capture_measurement_criteria
- To capture Measurement_Criterion for a Sensor_Measurement.

### PYR-RESP-0660 (`COMP-053-R03`) - determine_sensor_solution
- To determine a solution for use of Sensor_Resources that will meet given Requirements.

### PYR-RESP-0661 (`COMP-053-R04`) - determine_if_solution_remains_feasible
- To determine if a planned or ongoing Sensor_Solution remains feasible given current Capability.

### PYR-RESP-0662 (`COMP-053-R05`) - control_use_of_sensor
- To control the use of Sensor_Resources to obtain a measurement.

### PYR-RESP-0663 (`COMP-053-R06`) - capture_sensor_data
- To collect data from a Sensor_Resource (e.g. temperature from a thermometer or location of object in an area).

### PYR-RESP-0664 (`COMP-053-R07`) - identify_progress
- To identify progress against a Requirement.

### PYR-RESP-0665 (`COMP-053-R08`) - provide_sensor_data_feedback
- To provide the wider system with feedback data.

### PYR-RESP-0666 (`COMP-053-R09`) - update_resource_usages
- To update the status of the sensors usage of the platform's resources.

### PYR-RESP-0667 (`COMP-053-R10`) - assess_sensor_capability
- To assess the capability to perform sensing using Sensor_Resources, taking into account available resources and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0668 (`COMP-053-R11`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0669 (`COMP-053-R12`) - predict_capability_progression
- To predict the progression of Sensor_Resource capability over time and with use.

## COMP-054 - Signature (5.4.2.54)
### PYR-RESP-0670 (`COMP-054-R01`) - capture_requirements_for_signature_calculations
- To capture provided Signature requirements (e.g. determine own vehicle infrared emissions for a given Aspect) for the Signature calculations.

### PYR-RESP-0671 (`COMP-054-R02`) - capture_measurement_criteria_for_signature_calculations
- To capture provided Measurement_Criterion/criteria for Signatures (i.e. that the signature determination is within a specified margin of error).

### PYR-RESP-0672 (`COMP-054-R03`) - determine_object_emissions
- To determine the emissions of an Object in a given State.

### PYR-RESP-0673 (`COMP-054-R04`) - determine_object_reflections
- To determine the reflections from an Object in a given State from a given Aspect.

### PYR-RESP-0674 (`COMP-054-R05`) - determine_signature_for_provided_configuration
- To determine the Signature of an Object that would result from the Object implementing or enacting a proposed State.

### PYR-RESP-0675 (`COMP-054-R06`) - determine_configuration
- To determine a State that satisfies a required Signature level.

### PYR-RESP-0676 (`COMP-054-R07`) - determine_quality_of_deliverables
- To determine the quality of the deliverables provided by Signature during execution, measured against given requirements and the Measurement_Criterion/criteria.

### PYR-RESP-0677 (`COMP-054-R08`) - assess_signature_capability
- To assess the Capability to provide the Signature calculations taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0678 (`COMP-054-R09`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Signature assessment Capability.

### PYR-RESP-0679 (`COMP-054-R10`) - predict_capability_progression
- To predict the progression of Signature's Capability over time and with use.

## COMP-055 - Spatial Correction (5.4.2.55)
### PYR-RESP-0680 (`COMP-055-R01`) - capture_requirements_for_spatial_correction
- To capture provided Requirements for calculating Spatial_Corrections.

### PYR-RESP-0681 (`COMP-055-R02`) - capture_measurement_criteria
- To capture given Measurement_Criterion.

### PYR-RESP-0682 (`COMP-055-R03`) - determine_spatial_correction
- To determine the Spatial_Corrections resulting from a Physical_Effect.

### PYR-RESP-0683 (`COMP-055-R04`) - determine_quality_of_spatial_correction
- To determine the quality of the Spatial_Correction against the Measurement_Criterion.

### PYR-RESP-0684 (`COMP-055-R05`) - assess_capability_to_provide_spatial_correction
- To assess the Capability to determine Spatial_Corrections, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0685 (`COMP-055-R06`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0686 (`COMP-055-R07`) - predict_capability_progression
- To predict the progression of the Capability over time and with use.

## COMP-056 - Spectrum (5.4.2.56)
### PYR-RESP-0687 (`COMP-056-R01`) - capture_requirements
- To capture provided Requirements for Spectrum use.

### PYR-RESP-0688 (`COMP-056-R02`) - capture_constraints
- To capture provided constraints (e.g. EMCON).

### PYR-RESP-0689 (`COMP-056-R03`) - determine_allocation_solution
- To determine a Spectrum_Element_Allocation to satisfy Requirements and Interoperability_Criteria within the Spectrum_Constraints.

### PYR-RESP-0690 (`COMP-056-R04`) - determine_available_spectrum
- To determine parts of a Spectrum available for use.

### PYR-RESP-0691 (`COMP-056-R05`) - assess_spectrum_capability
- To assess the Capability to allocate Spectrum for use.

### PYR-RESP-0692 (`COMP-056-R06`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0693 (`COMP-056-R07`) - identify_whether_solution_remains_feasible
- To identify whether a planned or ongoing Spectrum_Element_Allocation remains feasible given currently available Spectrum and Spectrum_Constraints.

## COMP-057 - Storage (5.4.2.57)
### PYR-RESP-0694 (`COMP-057-R01`) - capture_requirements_for_sanitisation
- To capture provided Requirement(s) for sanitisation.

### PYR-RESP-0695 (`COMP-057-R02`) - capture_requirements_for_storage
- To capture provided Requirement(s) (e.g. desired effect or output, timing or balancing) for storage.

### PYR-RESP-0696 (`COMP-057-R03`) - capture_measurement_criteria_for_storage
- To capture given Measure_of_Achievement.

### PYR-RESP-0697 (`COMP-057-R04`) - capture_storage_constraints
- To capture Constraint(s) on the manner in which data is to be stored (e.g. security, accessibility or redundancy).

### PYR-RESP-0698 (`COMP-057-R05`) - determine_storage_solution
- To determine how to meet the given Requirement(s) and Constraint(s) for storage, e.g. moving data to balance the Storage_Space whilst observing security constraints.

### PYR-RESP-0699 (`COMP-057-R06`) - determine_if_storage_solution_remains_feasible
- To identify whether the planned or in progress Storage_Solution against a Requirement is still feasible given current or predicted Capability and conditions.

### PYR-RESP-0700 (`COMP-057-R07`) - deliver_storage_solution
- To deliver a Storage_Solution to meet the Requirement(s) within the Constraint(s).

### PYR-RESP-0701 (`COMP-057-R08`) - initiate_sanitisation_of_storage_space
- To initiate the sanitisation of a Storage_Space.

### PYR-RESP-0702 (`COMP-057-R09`) - identify_progress
- To identify the progress against the Requirement.

### PYR-RESP-0703 (`COMP-057-R10`) - assess_storage_capability
- To assess the Capability to provide storage taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0704 (`COMP-057-R11`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0705 (`COMP-057-R12`) - predict_storage_capability_progression
- To predict the progression of the storage Capability over time and with use.

## COMP-058 - Stores Release (5.4.2.58)
### PYR-RESP-0706 (`COMP-058-R01`) - capture_release_package_requirement
- To capture the Requirements for the release of Stores.

### PYR-RESP-0707 (`COMP-058-R02`) - capture_measurement_criteria
- To capture provided Measurement_Criterion which a Release_Schedule will be measured against.

### PYR-RESP-0708 (`COMP-058-R03`) - capture_provided_constraints
- To capture provided Constraints for stores release.

### PYR-RESP-0709 (`COMP-058-R04`) - determine_release_schedule
- To determine the Release_Schedule for Stores (when part of a Release_Package) with respect to provided Requirements and Constraints.

### PYR-RESP-0710 (`COMP-058-R05`) - determine_stores_releasability
- To determine which Stores are releasable.

### PYR-RESP-0711 (`COMP-058-R06`) - determine_release_packages
- To determine the Release_Packages (i.e. packages that do not compromise air vehicle safety) in response to a Requirement.

### PYR-RESP-0712 (`COMP-058-R07`) - identify_whether_solution_is_feasible
- To identify whether a Release_Schedule in progress remains feasible given current resources.

### PYR-RESP-0713 (`COMP-058-R08`) - identify_pre-conditions
- To identify Pre-conditions required to support the Release_Schedule.

### PYR-RESP-0714 (`COMP-058-R09`) - execute_release_solution
- To coordinate the execution of a Release_Schedule.

### PYR-RESP-0715 (`COMP-058-R10`) - determine_store_release_progress
- To determine the progress against the Release_Schedule (e.g. report whether Stores have hung or have left the platform).

### PYR-RESP-0716 (`COMP-058-R11`) - determine_quality_of_solution
- To determine the quality of a proposed Release_Schedule against given Requirements.

### PYR-RESP-0717 (`COMP-058-R12`) - assess_stores_release_capability
- To assess the Stores_Release_Capability taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0718 (`COMP-058-R13`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Stores_Release_Capability assessment.

### PYR-RESP-0719 (`COMP-058-R14`) - predict_stores_release_capability_progression
- To predict the progression of the Stores_Release_Capability over time and with use.

## COMP-059 - Susceptibility (5.4.2.59)
### PYR-RESP-0720 (`COMP-059-R01`) - capture_susceptibility_requirements
- To capture requirements for a Susceptibility assessment (i.e. predicted harm and/or required effect) for a given Engagement.

### PYR-RESP-0721 (`COMP-059-R02`) - capture_susceptibility_measurement_criteria
- To capture provided Measurement_Criterion for a Susceptibility assessment.

### PYR-RESP-0722 (`COMP-059-R03`) - determine_susceptibility
- To determine the Susceptibility of a Subject to the Offensive_Capability of an Aggressor.

### PYR-RESP-0723 (`COMP-059-R04`) - identify_subject_vulnerability
- To identify which Subject vulnerabilities an Aggressor could exploit.

### PYR-RESP-0724 (`COMP-059-R05`) - determine_required_effect_level
- To determine what level of Offensive_Capability an Aggressor must use to exploit a Subject's Vulnerability through a given medium.

### PYR-RESP-0725 (`COMP-059-R06`) - determine_susceptibility_quality
- To determine the quality of the Susceptibility assessment against given Measurement_Criterion/criteria.

### PYR-RESP-0726 (`COMP-059-R07`) - assess_susceptibility_capability
- To assess the Capability to determine Susceptibility taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0727 (`COMP-059-R08`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Susceptibility Capability assessment.

### PYR-RESP-0728 (`COMP-059-R09`) - predict_capability_progression
- To predict the progression of the Susceptibility Capability over time and with use.

<a id="comp-060"></a>
## COMP-060 - Tactical Objects (5.4.2.60)
### PYR-RESP-0729 (`COMP-060-R01`) - capture_object_interest_requirement_for_tactical_object
- To capture provided Object_Interest_Requirements (e.g. the scope of the information required and the frequency that it is reported) for Tactical_Objects.

### PYR-RESP-0730 (`COMP-060-R02`) - capture_measurement_criteria_for_tactical_object
- To capture provided Measurement_Criterion/criteria for Tactical_Objects.

### PYR-RESP-0731 (`COMP-060-R03`) - capture_tactical_objects_constraints
- To capture provided Constraints for Tactical_Objects solutions (e.g. do not process classified information or stop processing information derived from a specified source).

### PYR-RESP-0732 (`COMP-060-R04`) - identify_whether_requirement_is_achievable
- To identify whether an Object_Interest_Requirement is still achievable given current or predicted Capability and Constraints.

### PYR-RESP-0733 (`COMP-060-R05`) - determine_requirement_solution
- To determine a solution to Object_Interest_Requirements.

### PYR-RESP-0734 (`COMP-060-R06`) - determine_potential_objects
- To determine the potential for Tactical_Objects to exist at locations.

### PYR-RESP-0735 (`COMP-060-R07`) - determine_object_information_confidence
- To determine a level of confidence in the individual Tactical_Object's characteristics (e.g. behaviour, allegiance and association between tactical objects).

### PYR-RESP-0736 (`COMP-060-R08`) - determine_additional_information
- To determine additional information required to satisfy Object_Interest_Requirements, e.g. improved Tactical_Object confidence required.

### PYR-RESP-0737 (`COMP-060-R09`) - determine_object_relationships
- To determine the Relationships and dependencies between Tactical_Objects (e.g. if a radar is part of a specific vehicle, or if a specific vehicle is part of this formation and is the flight lead).

### PYR-RESP-0738 (`COMP-060-R10`) - estimate_object_behaviour
- To estimate the Behaviour exhibited by Tactical Objects, including individual Behaviour (e.g. that object is loitering) and Behaviour between Tactical Objects (e.g. that Exploiting Platform is tracking that tank, that Exploiting Platform is landing at that airfield). Behaviour also includes the operational state of a Tactical_Object (e.g. ready for operation, operational on ground, operational in air, or non-operational).

### PYR-RESP-0739 (`COMP-060-R11`) - identify_progress
- To identify the progress against an Object_Interest_Requirement.

### PYR-RESP-0740 (`COMP-060-R12`) - determine_quality_of_tactical_object_of_interest
- To determine the quality of the Tactical_Object_of_Interest provided by Tactical Objects during execution, measured against given Object_Interest_Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0741 (`COMP-060-R13`) - capture_object_information
- To capture information about Tactical_Objects, including but not limited to: their allegiance to organisations, kinematic behaviour (e.g. velocity, acceleration or path), location (e.g. at this position, within this region) and classification, including their type (e.g. surface ship or emitter), specific type (e.g. Type-45, Captor radar) and registration (e.g. HMS Daring).

### PYR-RESP-0742 (`COMP-060-R14`) - capture_object_probability_densities
- To capture the probability densities of particular Tactical_Objects within locations.

### PYR-RESP-0743 (`COMP-060-R15`) - assess_capability
- To assess the Capability to provide Tactical_Object information (e.g. determination and estimation of object characteristics) taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0744 (`COMP-060-R16`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the component's Capability (i.e. determination and estimation of object characteristics) assessment.

### PYR-RESP-0745 (`COMP-060-R17`) - predict_capability_progression
- To predict the progression of the Tactical Objects Capability over time and with use.

## COMP-061 - Target Engagement (5.4.2.61)
### PYR-RESP-0746 (`COMP-061-R01`) - capture_requirements_for_target_engagement
- To capture provided Requirements (e.g. Target, Engagement_Type, timing, and weapon type, if specified) for target engagement.

### PYR-RESP-0747 (`COMP-061-R02`) - capture_measurement_criteria_for_target_engagement
- To capture provided Measurement_Criterion/criteria (e.g. effectiveness and precision) that an Engagement_Solution and Engagements will be measured against.

### PYR-RESP-0748 (`COMP-061-R03`) - capture_constraints_for_target_engagement
- To capture provided Constraints for target engagement.

### PYR-RESP-0749 (`COMP-061-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current Target_Engagement_Resources and Constraints.

### PYR-RESP-0750 (`COMP-061-R05`) - determine_target_engagement_solution
- To determine an Engagement_Solution that meets the given Requirements within provided Constraints using available Target_Engagement_Resources.

### PYR-RESP-0751 (`COMP-061-R06`) - determine_predicted_quality_of_target_engagement_solution
- To determine the quality of the proposed Engagement_Solution against given Measurement_Criterion/criteria.

### PYR-RESP-0752 (`COMP-061-R07`) - identify_pre-conditions
- To identify Pre-conditions required to support target engagement.

### PYR-RESP-0753 (`COMP-061-R08`) - coordinate_target_engagement_solution
- To coordinate the execution of an Engagement_Solution by commanding Target_Engagement_Resources.

### PYR-RESP-0754 (`COMP-061-R09`) - identify_progress_of_target_engagement_solution
- To identify the progress of an Engagement_Solution against the Requirements.

### PYR-RESP-0755 (`COMP-061-R10`) - determine_actual_quality_of_outcome
- To determine the quality of the Engagement outcomes generated by executing an Engagement_Solution, measured against given Requirements and Measurement_Criterion/criteria.

### PYR-RESP-0756 (`COMP-061-R11`) - assess_target_engagement_capability
- To assess the Capability of the component to perform target engagement using available weapons and other Target_Engagement_Resources within given Constraints, taking into account system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0757 (`COMP-061-R12`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0758 (`COMP-061-R13`) - predict_progression_of_capability
- To predict the progression of the component's Capability to perform Engagement_Solutions over time and with use.

## COMP-062 - Tasks (5.4.2.62)
### PYR-RESP-0759 (`COMP-062-R01`) - capture_task_requirements
- To capture given Tasking requirements (e.g. task type, timing, importance of success, or risk profile).

### PYR-RESP-0760 (`COMP-062-R02`) - capture_measurement_criteria
- To capture given optimisation criteria for task solutions (e.g. quality, risk, or robustness).

### PYR-RESP-0761 (`COMP-062-R03`) - capture_system_constraints
- To capture given System_Constraints (e.g. budgets or operating restrictions).

### PYR-RESP-0762 (`COMP-062-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Tasking requirement is still achievable given current or predicted Composite_Capability and System_Constraints.

### PYR-RESP-0763 (`COMP-062-R05`) - determine_implementation_solution
- To determine a Sequence of Derived_Needs against given optimisation criteria.

### PYR-RESP-0764 (`COMP-062-R06`) - satisfy_dependencies_between_derived_needs
- To satisfy the dependencies between Derived_Needs by arranging their Sequence.

### PYR-RESP-0765 (`COMP-062-R07`) - coordinate_solution_enactment
- To coordinate the enactment of a response to a System_Stimulus via the execution of Derived_Needs in a Sequence.

### PYR-RESP-0766 (`COMP-062-R08`) - identify_progress_of_solution
- To identify the progress of a Sequence of Derived_Needs.

### PYR-RESP-0767 (`COMP-062-R09`) - evaluate_solution_quality
- To evaluate the quality of a Sequence of Derived_Needs against given optimisation criteria.

### PYR-RESP-0768 (`COMP-062-R10`) - determine_implementation_solution_cost
- To determine the cost of implementing a Sequence of Derived_Needs using the available composite capabilities.

### PYR-RESP-0769 (`COMP-062-R11`) - capture_contingency_definitions
- To capture given contingency definitions (e.g. the rules that constitute a Contingency_Situation and responses to that Contingency_Situation).

### PYR-RESP-0770 (`COMP-062-R12`) - assess_task_capability
- To assess the ability to respond to a System_Stimulus based upon available composite capabilities and observed anomalies.

### PYR-RESP-0771 (`COMP-062-R13`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the assessment of the ability to respond to a System_Stimulus.

### PYR-RESP-0772 (`COMP-062-R14`) - predict_capability_progression
- To predict the progression of the ability to respond to a System_Stimulus over time and with use.

## COMP-063 - Test (5.4.2.63)
### PYR-RESP-0773 (`COMP-063-R01`) - capture_test_requirements
- To capture Test_Requirements (e.g. system characteristics to test or a test methodology).

### PYR-RESP-0774 (`COMP-063-R02`) - capture_measurement_criteria
- To capture given Measurement_Criterion for the Test_Strategy.

### PYR-RESP-0775 (`COMP-063-R03`) - capture_test_constraints
- To capture test Constraints (e.g. operational limits or safety limits).

### PYR-RESP-0776 (`COMP-063-R04`) - identify_whether_requirement_remains_achievable
- To identify whether a Test_Requirement is still achievable given current or predicted Capability and Constraints.

### PYR-RESP-0777 (`COMP-063-R05`) - determine_test_which_meets_requirements
- To determine a Test_Strategy that meets the given Test_Requirement and Constraints using available resources.

### PYR-RESP-0778 (`COMP-063-R06`) - identify_test_pre-conditions
- To identify Pre-conditions required for a given Test_Strategy (e.g. required system configuration, power requirements, or resource moding).

### PYR-RESP-0779 (`COMP-063-R07`) - enact_test
- To carry out a Test_Strategy by executing individual Test_Actions (e.g. stimulating a component or requesting an item of equipment to initialise Built In Test (BIT)).

### PYR-RESP-0780 (`COMP-063-R08`) - determine_test_progress
- To determine the progress of a Test_Strategy against the Test_Requirement (e.g. percentage complete, time remaining, or phase of test).

### PYR-RESP-0781 (`COMP-063-R09`) - determine_test_outcome
- To determine the Test_Outcome of an enacted Test_Strategy.

### PYR-RESP-0782 (`COMP-063-R10`) - determine_cost_of_test_solution
- To determine the cost of a Test_Strategy (e.g. affected capabilities or cost of using resources).

### PYR-RESP-0783 (`COMP-063-R11`) - assess_capability
- To determine the test Capability (i.e. available tests) using available resources within given constraints, taking into account system health and observed anomalies.

### PYR-RESP-0784 (`COMP-063-R12`) - identify_missing_information
- To identify missing information that could improve the certainty or specificity of the test Capability assessment.

### PYR-RESP-0785 (`COMP-063-R13`) - predict_capability_progression
- To predict the progression of test Capability over time and with use.

## COMP-064 - Threats (5.4.2.64)
### PYR-RESP-0786 (`COMP-064-R01`) - capture_threat_assessment_requirement
- To capture the provided requirements for Threat_Assessments (e.g. the subject of the assessment (Threatenable_Object), region under consideration or frequency of assessments).

### PYR-RESP-0787 (`COMP-064-R02`) - capture_threat_assessment_constraints
- To capture provided Constraints for generating Threat_Assessments (e.g. limitations on threat types to consider or restrictions on information considered).

### PYR-RESP-0788 (`COMP-064-R03`) - identify_whether_requirement_remains_achievable
- To identify whether a Threat_Assessment is still achievable given current Capability and dependencies.

### PYR-RESP-0789 (`COMP-064-R04`) - determine_threat_assessment_solution
- To determine a solution (taking account of what information should be considered, the priority of Threatenable_Objects to assess and the trigger for assessment) which meets given requirements and Constraints for generating Threat_Assessments using available tactical information.

### PYR-RESP-0790 (`COMP-064-R05`) - assess_threats
- To generate Threat_Assessments for Threatenable_Objects based on the determined solution.

### PYR-RESP-0791 (`COMP-064-R06`) - assess_threat_assessment_capability
- To assess the Capability to generate Threat_Assessments, for particular Threatenable_Objects and Threat_Types (based on, for example, the available tactical information services and assessments).

### PYR-RESP-0792 (`COMP-064-R07`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Threat_Assessment generation Capability, i.e. capability to determine tactical information (e.g. observability or behaviour).

### PYR-RESP-0793 (`COMP-064-R08`) - predict_capability_progression
- To predict the progression of Threat_Assessment generation Capability over time and with use.

## COMP-065 - Trajectory Prediction (5.4.2.65)
### PYR-RESP-0794 (`COMP-065-R01`) - capture_requirements_for_trajectory_prediction
- To capture the Requirements to be satisfied by the Trajectory_Prediction_Solution.

### PYR-RESP-0795 (`COMP-065-R02`) - capture_measurement_criteria_for_trajectory_prediction
- To capture Measurement_Criterion/ criteria for Trajectory_Prediction_Solutions.

### PYR-RESP-0796 (`COMP-065-R03`) - identify_whether_requirement_remains_achievable
- To identify whether a Requirement is still achievable given current or predicted Capability and conditions.

### PYR-RESP-0797 (`COMP-065-R04`) - determine_trajectory_prediction_solution_for_object
- To determine the Trajectory_Prediction_Solution for an Object.

### PYR-RESP-0798 (`COMP-065-R05`) - capture_object_information
- To capture information about an Object's spatial state.

### PYR-RESP-0799 (`COMP-065-R06`) - capture_manoeuvring_characteristics
- To capture the Manoeuvring_Characteristics of an Object or type of object.

### PYR-RESP-0800 (`COMP-065-R07`) - identify_progress
- To identify the progress against a Requirement.

### PYR-RESP-0801 (`COMP-065-R08`) - assess_trajectory_prediction_capability
- To assess the Capability to predict trajectories, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0802 (`COMP-065-R09`) - identify_missing_information
- To identify missing information that could improve the accuracy or confidence level of the Trajectory prediction Capability assessment.

### PYR-RESP-0803 (`COMP-065-R10`) - predict_capability_progression
- To predict the progression of the component's Capability over time and with use.

## COMP-066 - Undercarriage (5.4.2.66)
### PYR-RESP-0804 (`COMP-066-R01`) - capture_undercarriage_requirement
- To capture provided Requirements for deployment or retraction of undercarriage.

### PYR-RESP-0805 (`COMP-066-R02`) - capture_undercarriage_constraints
- To capture undercarriage Constraints.

### PYR-RESP-0806 (`COMP-066-R03`) - determine_undercarriage_solution
- To determine an Undercarriage_Solution that meets the given Requirements and Constraints.

### PYR-RESP-0807 (`COMP-066-R04`) - determine_undercarriage_state
- To determine the Undercarriage_State (e.g. up, travelling, down or weight on or off wheels).

### PYR-RESP-0808 (`COMP-066-R05`) - identify_undercarriage_solution_in_progress_remains_feasible
- To identify whether an Undercarriage_Solution in progress remains feasible given current Capability.

### PYR-RESP-0809 (`COMP-066-R06`) - identify_pre_condition
- To identify Pre-conditions required to support the Undercarriage_Solution or an Undercarriage_Step.

### PYR-RESP-0810 (`COMP-066-R07`) - coordinate_undercarriage_movement
- To coordinate the deployment or retraction of the undercarriage to fulfil an undercarriage Requirement.

### PYR-RESP-0811 (`COMP-066-R08`) - identify_progress_of_undercarriage_solution
- To identify the progress of an Undercarriage_Solution against the Requirements.

### PYR-RESP-0812 (`COMP-066-R09`) - assess_undercarriage_capability
- To assess the Capability to determine the Undercarriage_State and to extend or retract the undercarriage taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0813 (`COMP-066-R10`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0814 (`COMP-066-R11`) - predict_capability_progression
- To predict the progression of the Undercarriage component Capability over time and with use.

## COMP-067 - User Accounts (5.4.2.67)
### PYR-RESP-0815 (`COMP-067-R01`) - validate_logon_attempt
- To determine whether a log on attempt is valid (by checking the supplied User_Identity and Credentials).

### PYR-RESP-0816 (`COMP-067-R02`) - validate_user_credentials
- To validate whether the supplied credentials for a User are valid.

### PYR-RESP-0817 (`COMP-067-R03`) - determine_user_status
- To determine User status information (e.g. logon status, specific equipment needs or clearance levels).

### PYR-RESP-0818 (`COMP-067-R04`) - determine_changes_to_user_accounts
- To add, update or delete User_Accounts according to specified rules (e.g. credential expiry or lock after failed login attempts).

### PYR-RESP-0819 (`COMP-067-R05`) - capture_user_identity
- To capture the User_Identity.

### PYR-RESP-0820 (`COMP-067-R06`) - capture_user_credentials
- To capture the Credentials used for the validation of Users (e.g. password or fingerprint).

### PYR-RESP-0821 (`COMP-067-R07`) - capture_user_ability
- To capture the Ability of a User (e.g. whether a user is trained in performing certain actions).

### PYR-RESP-0822 (`COMP-067-R08`) - capture_user_need
- To capture any User specific equipment User_Needs (e.g. wheelchair accessible workstation).

## COMP-068 - User Roles (5.4.2.68)
### PYR-RESP-0823 (`COMP-068-R01`) - capture_requirements_for_role_allocation_change
- To capture provided requirements for User to Role allocation, deallocation and handover requests.

### PYR-RESP-0824 (`COMP-068-R02`) - capture_constraints
- To capture the Constraints on the mapping of a User to a Role.

### PYR-RESP-0825 (`COMP-068-R03`) - determine_user_permissions
- To determine the permissions allocated to a User as part of a Role_Type (e.g. whether a user has permission to change file access permissions as part of their system administrative role).

### PYR-RESP-0826 (`COMP-068-R04`) - determine_operator_suitability_for_handover
- To determine the suitability of a User to receive a Role (e.g. if they are suitably qualified and the equipment is in place).

### PYR-RESP-0827 (`COMP-068-R05`) - determine_equipment_required_for_a_role
- To determine the equipment required to support a User in a Role_Type.

### PYR-RESP-0828 (`COMP-068-R06`) - determine_if_role_allocation_is_allowed
- To determine if allocation of a particular Role to a particular User is permissible.

### PYR-RESP-0829 (`COMP-068-R07`) - determine_if_allocation_change_is_feasible
- To determine if a planned or on-going allocation, deallocation, or handover of a User or users to/from a user Role is feasible given current Allocation_Capability and Constraints.

### PYR-RESP-0830 (`COMP-068-R08`) - allocate_roles
- To allocate, re-allocate and de-allocate Roles to Users.

### PYR-RESP-0831 (`COMP-068-R09`) - perform_role_handover
- To perform the handover of Roles between Users.

### PYR-RESP-0832 (`COMP-068-R10`) - assess_capability
- To assess the Allocation_Capability taking account of system health, observed anomalies, and availability (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0833 (`COMP-068-R11`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Allocation_Capability assessment.

### PYR-RESP-0834 (`COMP-068-R12`) - predict_capability_progression
- To predict the progression of Allocation_Capability over time and with use.

## COMP-069 - Vehicle External Environment (5.4.2.69)
### PYR-RESP-0835 (`COMP-069-R01`) - determine_properties
- To determine an Environmental_Property of the immediate External_Environment surrounding a Vehicle (e.g. temperature, static pressure, indicated airspeed or pressure altitude).

### PYR-RESP-0836 (`COMP-069-R02`) - determine_required_correction_factors
- To apply Correction_Factors to received sensor data.

### PYR-RESP-0837 (`COMP-069-R03`) - identify_pre-conditions
- To identify pre-conditions necessary to determine an Environmental_Property of the immediate External_Environment surrounding a Vehicle, e.g. to identify a required vehicle configuration.

### PYR-RESP-0838 (`COMP-069-R04`) - capture_reference_datum
- To capture appropriate Reference_Datum/data from which environmental relationships should be calculated.

### PYR-RESP-0839 (`COMP-069-R05`) - assess_capability
- To identify the Capability to carry out Environmental_Property determination, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0840 (`COMP-069-R06`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0841 (`COMP-069-R07`) - predict_capability_progression
- To predict the progression of the component's Capability over time and with use.

<a id="comp-070"></a>
## COMP-070 - Vehicle Guidance (5.4.2.70)
### PYR-RESP-0842 (`COMP-070-R01`) - capture_trajectory_requirements
- To capture given Trajectory_Requirements.

### PYR-RESP-0843 (`COMP-070-R02`) - capture_measurement_criteria
- To capture Measurement_Criterion of the trajectory.

### PYR-RESP-0844 (`COMP-070-R03`) - capture_vehicle_motion_constraints
- To capture given Vehicle Movement_Constraints (e.g. bank angle constraint).

### PYR-RESP-0845 (`COMP-070-R04`) - ensure_solution_validity
- To ensure that all necessary Validity_Rules have been adhered to prior to enactment of a Planned_Trajectory (e.g. spatial de-confliction).

### PYR-RESP-0846 (`COMP-070-R05`) - ensure_trajectory_continuity
- To ensure all adjoining Planned_Trajectory segments are continuous in terms of both location and attitude before being executed (i.e. there are no instantaneous changes).

### PYR-RESP-0847 (`COMP-070-R06`) - ensure_solution_flow
- To ensure that whilst in motion there is always a Planned_Trajectory for execution. This includes planned trajectories for which there is no Trajectory_Requirement to either segue between planned trajectories, which satisfy Trajectory_Requirements, or to provide fall back guidance solutions in the absence of Trajectory_Requirements for the future.

### PYR-RESP-0848 (`COMP-070-R07`) - determine_planned_vehicle_trajectory
- To determine a Planned_Trajectory to achieve Trajectory_Requirements within defined Movement_Constraints and Validity_Rules.

### PYR-RESP-0849 (`COMP-070-R08`) - issue_vehicle_control_commands
- To execute the selected Planned_Trajectory by issuing Motion_Commands (e.g. attitude and speed or thrust level).

### PYR-RESP-0850 (`COMP-070-R09`) - identify_whether_requirement_remains_achievable
- To identify whether a Trajectory_Requirement is still achievable given current or predicted Guidance_Capability, Performance_Guides and Movement_Constraints.

### PYR-RESP-0851 (`COMP-070-R10`) - determine_trajectory_requirement_progress
- To determine the progress of a Vehicle against the Trajectory_Requirement.

### PYR-RESP-0852 (`COMP-070-R11`) - determine_predicted_quality_of_deliverables
- To determine the predicted quality of the Planned_Trajectory against given Measurement_Criterion/criteria.

### PYR-RESP-0853 (`COMP-070-R12`) - identify_vehicle_trajectory_divergence
- To identify Trajectory_Divergence of the Vehicle.

### PYR-RESP-0854 (`COMP-070-R13`) - determine_capability
- To determine the Guidance_Capability of the Vehicle, taking into account observed anomalies.

### PYR-RESP-0855 (`COMP-070-R14`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Guidance_Capability assessment.

### PYR-RESP-0856 (`COMP-070-R15`) - predict_vehicle_guidance_capability
- To predict the progression of Guidance_Capability over time and with use.

## COMP-071 - Vehicle Performance (5.4.2.71)
### PYR-RESP-0857 (`COMP-071-R01`) - determine_performance_envelope
- To determine and actively manage the suitable Performance_Envelope for a defined Vehicle_Activity and/or Vehicle_Configuration and identify conflicting demands which prevent a suitable Performance_Envelope from being determined based on the applicable Performance_Regimes and constraints.

### PYR-RESP-0858 (`COMP-071-R02`) - determine_applicable_values
- To determine applicable Performance_Parameter values, which apply to a Performance_Regime or Performance_Envelope, for combinations of Vehicle_Activity, External_Condition and Vehicle_Configuration.

### PYR-RESP-0859 (`COMP-071-R03`) - determine_vehicle_configuration
- To determine Vehicle_Configurations for which a combination of Performance_Parameter values is allowable.

### PYR-RESP-0860 (`COMP-071-R04`) - manage_performance_regime
- To determine and actively manage the Performance_Regime(s) in response to Vehicle_Activity, Vehicle_Configuration and External_Conditions and to identify conflicts which prevent an allowable Performance_Regime being determined.

### PYR-RESP-0861 (`COMP-071-R05`) - capture_performance_demands
- To capture all demands that will impact the Performance_Envelope.

### PYR-RESP-0862 (`COMP-071-R06`) - capture_vehicle_configuration
- To capture the Vehicle_Configuration.

### PYR-RESP-0863 (`COMP-071-R07`) - capture_vehicle_activity
- To capture any Vehicle_Activity that will affect vehicle performance.

### PYR-RESP-0864 (`COMP-071-R08`) - capture_conditions
- To capture External_Conditions that affect vehicle performance.

### PYR-RESP-0865 (`COMP-071-R09`) - assess_capability
- To assess the Capability to determine applicable Performance_Envelopes, identify Performance_Parameter constraints and answer queries, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0866 (`COMP-071-R10`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Vehicle Performance Capability assessment.

### PYR-RESP-0867 (`COMP-071-R11`) - predict_capability_progression
- To predict the progression of the component's Capability over time and with use.

## COMP-072 - Vehicle Stability and Control (5.4.2.72)
### PYR-RESP-0868 (`COMP-072-R01`) - capture_vehicle_control_requirements
- To capture Vehicle_Control_Requirements (e.g. attitude or velocity).

### PYR-RESP-0869 (`COMP-072-R02`) - capture_limits
- To capture given Stability_Control_Limits.

### PYR-RESP-0870 (`COMP-072-R03`) - identify_whether_requirement_remains_achievable
- To identify whether a Vehicle_Control_Requirement is still achievable given current or predicted Capability and Stability_Control_Limits.

### PYR-RESP-0871 (`COMP-072-R04`) - determine_control_effector_commands
- To determine Control_Effector_Commands (e.g. control surface movement or thrust demand) to achieve Vehicle_Control_Requirements.

### PYR-RESP-0872 (`COMP-072-R05`) - fulfil_requirement
- To fulfil a Vehicle_Control_Requirement by executing the planned Control_Effector_Commands.

### PYR-RESP-0873 (`COMP-072-R06`) - assess_capability
- To assess the Capability to control the Vehicle, taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage, or ageing).

### PYR-RESP-0874 (`COMP-072-R07`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the Capability assessment.

### PYR-RESP-0875 (`COMP-072-R08`) - predict_capability_progression
- To predict the progression of the component's Capability over time and with use.

## COMP-073 - Weather (5.4.2.73)
### PYR-RESP-0876 (`COMP-073-R01`) - capture_weather_picture_requirements
- To capture provided Requirements for the provision of a Weather_Picture.

### PYR-RESP-0877 (`COMP-073-R02`) - capture_measurement_criteria
- To capture provided Measurement_Criterion/criteria for the Weather_Picture, e.g. confidence of prediction.

### PYR-RESP-0878 (`COMP-073-R03`) - capture_constraints_for_weather_picture
- To capture Constraints relating to which Sources can be used for Weather_Types and Measurement_Types.

### PYR-RESP-0879 (`COMP-073-R04`) - determine_relevant_weather_information
- To determine the Weather_Picture at particular times (or time windows) and locations based on forecasts and Measurements.

### PYR-RESP-0880 (`COMP-073-R05`) - capture_measurements
- To capture Measurements from which Weather_Conditions can be determined, e.g. inputs from sensors.

### PYR-RESP-0881 (`COMP-073-R06`) - capture_weather_condition_information
- To capture information on Weather_Conditions, e.g. forecasts.

### PYR-RESP-0882 (`COMP-073-R07`) - determine_quality_of_weather_picture
- To determine the quality of the Weather_Picture against given Measurement_Criterion/criteria.

### PYR-RESP-0883 (`COMP-073-R08`) - assess_weather_capability
- To assess the Capability to determine the overall Weather_Picture taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### PYR-RESP-0884 (`COMP-073-R09`) - identify_missing_information
- To identify missing information which could improve the certainty or specificity of a Weather_Picture, e.g. a more recent forecast.

### PYR-RESP-0885 (`COMP-073-R10`) - predict_capability_progression
- To predict the progression of the Capability to determine the overall Weather_Picture over time and with use.


