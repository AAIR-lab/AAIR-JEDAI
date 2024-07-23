class Effect(object):

	def __init__(self,pos_predicates,neg_predicates=None):
		self.list_positive_predicates = pos_predicates
		self.list_negative_predicates = neg_predicates

	def getPositivePredicates(self):
		return self.list_positive_predicates

	def getNegativePredicates(self):
		return self.list_negative_predicates

	
	