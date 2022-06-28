Blockly.Blocks['start_block'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Start:");
      this.setNextStatement(true, null);
      this.setColour(230);
   this.setTooltip("Attach action blocks here.");
   this.setHelpUrl("");
    }
};
