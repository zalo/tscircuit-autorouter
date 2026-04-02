import { HighDensityInteractiveNodeDebugger } from "lib/testing/HighDensityInteractiveNodeDebugger"
import cmn79NodeData from "./cmn_79-nodeWithPortPoints.json" with {
  type: "json",
}

export default () => {
  return (
    <div className="p-2">
      <div className="mb-2 text-sm">
        bugreport46 extracted failing high-density node `cmn_79`
      </div>
      <div className="mb-3 text-xs text-gray-700">
        6 paired connections, 12 port points, all constrained to `z=1` with
        `availableZ: [1]` in a `1.65mm x 6.57mm` node.
      </div>
      <HighDensityInteractiveNodeDebugger
        nodeWithPortPoints={cmn79NodeData.nodeWithPortPoints}
      />
    </div>
  )
}
