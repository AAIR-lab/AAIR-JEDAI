
function setBlockTransparency(block, transparency) {

    block.pathObject.svgPath.style["fillOpacity"] = transparency;
    block.pathObject.svgPathDark.style["fillOpacity"] = transparency;
}

function setBlockTransparent(block) {

    setBlockTransparency(block, 0.3);
}


function setBlockOpaque(block) {

    setBlockTransparency(block, 1);
}

function setAllBlocksTransparent() {

    blocks = workspace.getAllBlocks();
    console.assert(blocks[0].type == "start_block");
    for (i = 1; i < blocks.length; i++) {
    
        setBlockTransparent(blocks[i])
    }
}

function setUnconnectedBlocksAsTransparent() {

    setAllBlocksTransparent();    

    blocks = workspace.getAllBlocks();
    start_block = blocks[0];
    children = start_block.getChildren();
    
    // The start block is always marked opaque.
    total_marked_opaque = 1
    while (children.length > 0) {
    
        console.assert(children.length == 1);
        child = children[0];
        setBlockOpaque(child);
        total_marked_opaque += 1;
        children = child.getChildren();
    }

    // Also, bring the start block to the front.
    start_block.bringToFront();

    if (total_marked_opaque > 1 && total_marked_opaque == blocks.length) {

        return false
    }
    else {

        return true
    }
    
}