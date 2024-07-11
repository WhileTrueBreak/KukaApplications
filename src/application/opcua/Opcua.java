package application.opcua;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ThreadPoolExecutor;

import com.prosysopc.ua.ApplicationIdentity;
import com.prosysopc.ua.UserIdentity;
import com.prosysopc.ua.client.MonitoredDataItem;
import com.prosysopc.ua.client.Subscription;
import com.prosysopc.ua.client.SubscriptionAliveListener;
import com.prosysopc.ua.client.SubscriptionNotificationListener;
import com.prosysopc.ua.client.UaClient;
import com.prosysopc.ua.stack.builtintypes.LocalizedText;
import com.prosysopc.ua.stack.builtintypes.NodeId;
import com.prosysopc.ua.stack.builtintypes.UnsignedInteger;
import com.prosysopc.ua.stack.common.ServiceResultException;
import com.prosysopc.ua.stack.core.ApplicationDescription;
import com.prosysopc.ua.stack.core.ApplicationType;
import com.prosysopc.ua.stack.core.Identifiers;
import com.prosysopc.ua.stack.core.MonitoringMode;
import com.prosysopc.ua.stack.core.ReferenceDescription;
import com.prosysopc.ua.stack.transport.security.SecurityMode;
import com.prosysopc.ua.stack.utils.StackUtils;

public class Opcua {
	
	public static final UnsignedInteger ATTRIB_ID= UnsignedInteger.valueOf(13);
	
	private UaClient client;
	private OpcuaDataListener dataListener;
	private Subscription subscription;
	private SubscriptionAliveListener subscriptionAliveListener;
	private SubscriptionNotificationListener subscriptionNotificationListener;

	private Map<String, NodeId> pathReadableNodeMapping;
	private Map<String, NodeId> pathWritableNodeMapping;
	
	public Opcua(String address) throws Exception {
		client = new UaClient(address);
		client.setSecurityMode(SecurityMode.NONE);
		initialise();
		client.setUserIdentity(new UserIdentity("Admin", "password"));
		client.connect();
		client.getAddressSpace().setMaxReferencesPerNode(1000);
		client.getAddressSpace().setReferenceTypeId(Identifiers.HierarchicalReferences);
		
		dataListener = new OpcuaDataListener();
		
		subscriptionAliveListener = new OpcuaSubscriptionAliveListener();
		subscriptionNotificationListener = new OpcuaSubscriptionNotificationListener();
		subscription = new Subscription();
		subscription.addAliveListener(subscriptionAliveListener);
		subscription.addNotificationListener(  subscriptionNotificationListener);
		subscription.setPublishingInterval(10);
		client.addSubscription(subscription);
		
		pathReadableNodeMapping = new HashMap<String, NodeId>();
		pathWritableNodeMapping = new HashMap<String, NodeId>();
	}

	protected void initialise() {
		ApplicationDescription applicationDescription = new ApplicationDescription();
		applicationDescription.setApplicationName(new LocalizedText("RobotController", Locale.ENGLISH));
		applicationDescription.setApplicationUri("RobotController");
		applicationDescription.setProductUri("RobotController");
		applicationDescription.setApplicationType(ApplicationType.Client);
		final ApplicationIdentity identity = new ApplicationIdentity();
		identity.setApplicationDescription(applicationDescription);
		client.setApplicationIdentity(identity);
	}

	public boolean addReadableNode(String nodePath) {
		String[] splitPath = nodePath.split("/");
		NodeId nextId = Identifiers.RootFolder;
		List<ReferenceDescription> references;
		for(String step:splitPath) {
			try {
				references = client.getAddressSpace().browse(nextId);
				nextId = findNode(client, step, references);
			} catch (Exception e) {
				e.printStackTrace();
				return false;
			}
		}
		
		MonitoredDataItem dataItem = new MonitoredDataItem(nextId, ATTRIB_ID, MonitoringMode.Reporting, subscription.getPublishingInterval());
		dataItem.setDataChangeListener(dataListener);
		try {
			dataItem.setDataChangeFilter(null);
			subscription.addItem(dataItem);
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
		pathReadableNodeMapping.put(nodePath, nextId);
		return true;
	}
	
	public boolean addWritableNode(String nodePath) {
		String[] splitPath = nodePath.split("/");
		NodeId nextId = Identifiers.RootFolder;
		List<ReferenceDescription> references;
		for(String step:splitPath) {
			try {
				references = client.getAddressSpace().browse(nextId);
				nextId = findNode(client, step, references);
			} catch (Exception e) {
				e.printStackTrace();
				return false;
			}
		}
		pathWritableNodeMapping.put(nodePath, nextId);
		return true;
	}
	
	public boolean hasNodeUpdated(String nodePath) {
		return dataListener.hasUpdated(pathReadableNodeMapping.get(nodePath));
	}
	
	public <T> T readNode(String nodePath, Class<T> type) {
		@SuppressWarnings("unchecked")
		T value = (T) dataListener.getNodeValue(pathReadableNodeMapping.get(nodePath));
		return value;
	}
	
	public <T> boolean writeNode(String nodePath, T value) {
		try {
			client.writeAttribute(pathWritableNodeMapping.get(nodePath), ATTRIB_ID, value);
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
		return true;
	}
	
	protected NodeId findNode(UaClient client, String comp, List<ReferenceDescription> references) throws ServiceResultException {
		for(ReferenceDescription reference:references) {
			if(reference.getDisplayName().toString().contains((comp))) {
				return client.getAddressSpace().getNamespaceTable().toNodeId(reference.getNodeId());
			}
		}
		return null;
	}
	
	public void disconnect() {
		client.disconnect();
	}
	
	public void shutdown() {
		client.disconnect();
		ThreadPoolExecutor blockingPool = (ThreadPoolExecutor) StackUtils.getBlockingWorkExecutor();
		ThreadPoolExecutor nonblockingPool = (ThreadPoolExecutor) StackUtils.getNonBlockingWorkExecutor();
		StackUtils.shutdown();
		while(blockingPool.getPoolSize() != 0 || nonblockingPool.getPoolSize() != 0);
	}
	
}
