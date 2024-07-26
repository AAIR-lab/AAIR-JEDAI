import copy
import os

import yaml

import config


def create_lattice():
    dest = config.LATTICE_DOCUMENT_FILE

    with open(config.DOMAIN_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    
    # print("LATTICE DATA = ",data)

    data = data.split("(:")[3].split("predicates")[1].replace(' ', '').split(")")
    preds = []
    for x in data:
        l = x.split("?")[0].split("(")
        if len(l) > 1:
            preds.append(l[1].replace(' ', ''))

    if len(preds) == 0:
        # TODO size?
        for ind in range(size):
            preds.append("p" + str(ind))
    size = len(preds)
    nodes = ["c" + str(i) for i in range(pow(2, size))]

    # doubt shouldnt init node start with c0 - the most concrete node
    # here c1 is being double counted and has a edge to itself
    # lattice_dict = {'Init': 'c1', 'Nodes': nodes, 'Edge_map': {}, 'Inv_edge_map': {}, 'Sup': "NAN", 'Node_map': {}}
    # lattice_dict = {'Init': 'c1', 'Nodes': nodes[1:], 'Edge_map': {}, 'Inv_edge_map': {}, 'Sup': "NAN", 'Node_map': {}}
    lattice_dict = {'Init': 'c0', 'Nodes': nodes[0:], 'Edge_map': {}, 'Inv_edge_map': {}, 'Sup': "NAN", 'Node_map': {}}

    # node_map = {'c1': set(i for i in preds)}
    node_map = {'c0': set(i for i in preds)}

    tot_key = '_'.join(sorted(preds))

    # reverse_node_map = {tot_key: 'c1'}
    reverse_node_map = {tot_key: 'c0'}

    unused_nodes = copy.deepcopy(nodes[1:])
    # last_level = ['c1']
    last_level = ['c0']

    edge_map = {}
    inv_edge_map = {}
    while unused_nodes:
        new_level = set()
        for node in last_level:
            pred_set = node_map[node]
            for p in pred_set:
                new_pred_set = pred_set - set([p])
                p_key = '_'.join(sorted(new_pred_set))
                try:
                    new_node = reverse_node_map[p_key]
                except:
                    new_node = unused_nodes.pop(0)
                    reverse_node_map[p_key] = new_node
                    node_map[new_node] = new_pred_set
                new_level.add(new_node)
                try:
                    lattice_dict['Edge_map'][node][p] = new_node
                except:
                    lattice_dict['Edge_map'][node] = {}
                    lattice_dict['Edge_map'][node][p] = new_node
                try:
                    lattice_dict['Inv_edge_map'][new_node][p] = node
                except:
                    lattice_dict['Inv_edge_map'][new_node] = {}
                    lattice_dict['Inv_edge_map'][new_node][p] = node
                    # edge_map[node+'#'+p] = new_node
        last_level = new_level
    lattice_dict['Sup'] = list(last_level)[0]
    rev_node_map = {}
    for nd in node_map:
        rev_node_map[nd] = set(preds) - node_map[nd]
    lattice_dict['Node_map'] = rev_node_map

    print(lattice_dict)

    with open(dest, 'w') as t_fd:
        yaml.dump({'Lattice': lattice_dict}, t_fd)
