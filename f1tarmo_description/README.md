# f1tarmo_description

## TODO
The README for this package should detail:
- The purpose and origin of each URDF/xacro file it contains, a small
  explanation for each.
- A brief explanation of the rationale behind the model and resulting tf tree,
  woes with onshape-to-robot, etc. Basically, explain to them why the base link
  isn't the root (if I end up doing that). This would also be a good place to
  describe REP 105...

For the URDF generated from CAD via onshape-to-robot, I'm thinking that I should
just drop a link in there to the document in the doc folder that describes the
workflow for generating this from the onshape design. Either that, or that
document lives in here. Not sure which makes more sense.

If you're just building a "standard" f1tarmo, you don't care, and don't need
to carry out those steps. For that reason, I think it might make the most sense
to just document that process here, as that is really specific to generating the
contents of this package, or modifying/extending it if you are making a
different version of the f1tarmo / extending it. Then, in the main docs, I can
just link to this document.

## Version History

Todo: Turn this into a table, and look for some examples online with similar
version tracking.
- 0.0.1: Initial base f1tarmo description created with realsense mount.
- 0.0.2: RealSense D435i URDF included in the f1tarmo's description, joined to
  the f1tarmo on the realsense mount.