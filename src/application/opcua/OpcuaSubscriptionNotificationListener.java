package application.opcua;

import com.prosysopc.ua.client.MonitoredDataItem;
import com.prosysopc.ua.client.MonitoredEventItem;
import com.prosysopc.ua.client.Subscription;
import com.prosysopc.ua.client.SubscriptionNotificationListener;
import com.prosysopc.ua.stack.builtintypes.DataValue;
import com.prosysopc.ua.stack.builtintypes.DiagnosticInfo;
import com.prosysopc.ua.stack.builtintypes.ExtensionObject;
import com.prosysopc.ua.stack.builtintypes.StatusCode;
import com.prosysopc.ua.stack.builtintypes.UnsignedInteger;
import com.prosysopc.ua.stack.builtintypes.Variant;
import com.prosysopc.ua.stack.core.NotificationData;

public class OpcuaSubscriptionNotificationListener implements SubscriptionNotificationListener{

	@Override
	public void onBufferOverflow(Subscription s, UnsignedInteger seqNum, ExtensionObject[] notifData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onDataChange(Subscription s, MonitoredDataItem item, DataValue newValue) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onError(Subscription s, Object notification, Exception exception) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onEvent(Subscription s, MonitoredEventItem item, Variant[] eventFields) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public long onMissingData(Subscription s, UnsignedInteger lastSeqNum, long seqNum, long newSeqNum, StatusCode serviceResult) {
		// TODO Auto-generated method stub
		return newSeqNum;
	}

	@Override
	public void onNotificationData(Subscription s, NotificationData notification) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStatusChange(Subscription s, StatusCode oldStatus, StatusCode newStatus, DiagnosticInfo info) {
		// TODO Auto-generated method stub
		
	}

}
