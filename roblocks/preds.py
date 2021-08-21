import config

with open(config.DOMAIN_DOCUMENT_FILE, 'r') as file:
    data = file.read().replace('\n', '')
data = data.split("(:predicates")[1].split(");")[0].split(")")
preds = []
for x in data:
    l = x.split("?")[0].split("(")
    if len(l) > 1:
        preds.append(l[1].replace(' ', ''))
print(preds)
