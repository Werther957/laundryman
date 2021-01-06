:- register_ros_package(knowrob_maps).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_common).


:- consult('instance_utils').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://laundryman/owl/ssy235Ontology.owl').
:- rdf_db:rdf_register_ns(ssy235Ontology, 'http://www.semanticweb.org/xm/ontologies/2020/11/ssy235Ontology.owl#', [keep(true)]).
%:- owl_parser:owl_parse('package://laundryman/owl/ssy235Ontology.owl').
%:- rdf_db:rdf_register_ns(ssy235Ontology, 'http://www.chalmers.se/ontologies/ssy235Ontology.owl#', [keep(true)]).