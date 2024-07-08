package application.opcua;

import java.util.HashMap;
import java.util.Map;

import com.prosysopc.ua.client.MonitoredDataItem;
import com.prosysopc.ua.client.MonitoredDataItemListener;
import com.prosysopc.ua.stack.builtintypes.DataValue;
import com.prosysopc.ua.stack.builtintypes.NodeId;

public class OpcuaDataListener implements MonitoredDataItemListener{

	private Map<NodeId, Object> nodeValues;
	private Map<NodeId, Boolean> hasUpdatedMap;
	
	public OpcuaDataListener() {
		nodeValues = new HashMap<NodeId, Object>();
		hasUpdatedMap = new HashMap<NodeId, Boolean>();
	}
	
	@Override
	public void onDataChange(MonitoredDataItem sender, DataValue preValue, DataValue value) {
		nodeValues.put(sender.getNodeId(), value.getValue().getValue());
		hasUpdatedMap.put(sender.getNodeId(), true);
	}
	
	public Object getNodeValue(NodeId id) {
		hasUpdatedMap.put(id, false);
		if(!nodeValues.containsKey(id)) return null;
		return nodeValues.get(id);
	}
	
	public boolean hasUpdated(NodeId id) {
		if(!hasUpdatedMap.containsKey(id)) return false;
		return hasUpdatedMap.get(id);
	}

}
