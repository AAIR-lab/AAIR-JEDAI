class Function(object):

	def __init__(self, arguments ):
		'''
		Input : list of tuples ( argumnet_name , argumnet_type , argument_value) 
				MUST BE ORDERED 
		'''
		self.list_arguments = arguments

	def setArgumentValues(self,arguments):
		self.dict_arguments = arguments

	def getArgumentValues(self):
		return self.dict_arguments
