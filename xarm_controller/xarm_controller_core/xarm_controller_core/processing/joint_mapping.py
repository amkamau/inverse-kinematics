class JointMapper:
    def __init__(self, joint_order=None, directions=None, offsets=None):
        self.joint_order = joint_order
        self.directions = directions or {}
        self.offsets = offsets or {}

    def map_positions(self, joint_names, positions):
        """
        Map MoveIt joint positions into the arm's expected joint order
        and apply optional direction inversions and offsets.

        Positions are assumed to be in radians.
        """
        positions_by_name = dict(zip(joint_names, positions))

        if self.joint_order is None:
            output_names = list(joint_names)
        else:
            output_names = self.joint_order

        output_positions = []

        for name in output_names:
            position = positions_by_name[name]
            direction = self.directions.get(name, 1.0)
            offset = self.offsets.get(name, 0.0)

            output_positions.append(direction * position + offset)

        return output_names, output_positions

