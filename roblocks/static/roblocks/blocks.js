
Blockly.Blocks['start_block'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Start:");
      this.setNextStatement(true, null);
      this.setColour(230);
   this.setTooltip("");
   this.setHelpUrl("");
    }
};
    
var unstack_action_JSON = {
    "type": "Empty_gripper",
    "message0": "take %1 off %2",
    "args0": [
      {"type": "input_value", "name": "DELTA", "check": "Object"},
      {"type": "input_value", "name": "DELTA", "check": "Object"}
    ],
    "previousStatement": "Full_gripper",
    "nextStatement": "Empty_gripper",
    "colour": "#17A991"
  };

Blockly.Blocks['unstack_action'] = {
    init: function() {
      this.jsonInit(unstack_action_JSON);
      // Assign 'this' to a variable for use in the tooltip closure below.
      var thisBlock = this;
      this.setTooltip(function() {
        return 'Add a number to variable "%1".'.replace('%1',
            thisBlock.getFieldValue('VAR'));
      });
    }
  };


var stack_action_JSON = {
    "type": "Full_gripper",
    "message0": "stack %1 on top of %2",
    "args0": [
      {"type": "input_value", "name": "DELTA", "check": "Object"},
      {"type": "input_value", "name": "DELTA", "check": "Object"}
    ],
    "previousStatement": "Empty_gripper",
    "nextStatement": "Full_gripper",
    "colour": "#428ae6"
  };

Blockly.Blocks['stack_action'] = {
    init: function() {
      this.jsonInit(stack_action_JSON);
      // Assign 'this' to a variable for use in the tooltip closure below.
      var thisBlock = this;
      this.setTooltip(function() {
        return 'Add a number to variable "%1".'.replace('%1',
            thisBlock.getFieldValue('VAR'));
      });
    }
  };




var grap_action_JSON = {
    "type": "Empty_gripper",
    "message0": "grap %1 using %2 ",
    "args0": [
      {"type": "input_value", "name": "DELTA", "check": "Object"},
      {"type": "input_value", "name": "DELTA", "check": "Subject"}
    ],
    "inputsInline": true,
    "previousStatement": "Full_gripper",
    "nextStatement": "Empty_gripper",
    "colour": 330
  };

Blockly.Blocks['grap_action'] = {
    init: function() {
      this.jsonInit(grap_action_JSON);
      // Assign 'this' to a variable for use in the tooltip closure below.
      var thisBlock = this;
      this.setTooltip(function() {
        return 'Add a number to variable "%1".'.replace('%1',
            thisBlock.getFieldValue('VAR'));
      });
    }
  };

var put_action_JSON = {
    "type": "Full_gripper",
    "message0": "put %1 down using %2 ",
    "args0": [
      {"type": "input_value", "name": "DELTA", "check": "Object"},
      {"type": "input_value", "name": "DELTA", "check": "Subject"}
    ],
    "inputsInline": true,
    "previousStatement": "Empty_gripper",
    "nextStatement": "Full_gripper",
    "colour": "#6a16d5"
  };

Blockly.Blocks['put_action'] = {
    init: function() {
      this.jsonInit(put_action_JSON);
      // Assign 'this' to a variable for use in the tooltip closure below.
      var thisBlock = this;
      this.setTooltip(function() {
        return 'Add a number to variable "%1".'.replace('%1',
            thisBlock.getFieldValue('VAR'));
      });
    }
  };


Blockly.Blocks['green_block'] = {
    init: function() {
          this.appendDummyInput()
              .setAlign(Blockly.ALIGN_RIGHT)
              .appendField("Green Block");
          this.setInputsInline(true);
          this.setOutput(true, "Object");
          this.setColour(120);
       this.setTooltip("test");
       this.setHelpUrl("");
        }
      };

Blockly.Blocks['red_block'] = {
    init: function() {
          this.appendDummyInput()
              .setAlign(Blockly.ALIGN_RIGHT)
              .appendField("Red Block");
          this.setInputsInline(true);
          this.setOutput(true, "Object");
          this.setColour("#fe3622");
       this.setTooltip("test");
       this.setHelpUrl("");
        }
      };

Blockly.Blocks['blue_block'] = {
    init: function() {
          this.appendDummyInput()
              .setAlign(Blockly.ALIGN_RIGHT)
              .appendField("Blue Block");
          this.setInputsInline(true);
          this.setOutput(true, "Object");
          this.setColour(225);
       this.setTooltip("test");
       this.setHelpUrl("");
        }
      };

Blockly.Blocks['gripper'] = {
    init: function() {
          this.appendDummyInput()
              .setAlign(Blockly.ALIGN_RIGHT)
              .appendField("Gripper");
          this.setInputsInline(true);
          this.setOutput(true, "Subject");
          this.setColour("#D3DF2D");
       this.setTooltip("test");
       this.setHelpUrl("");
        }
      };


