// RHMesh.cpp
//
// Define addressed datagram
//
// Part of the Arduino RH library for operating with HopeRF RH compatible transceivers
// (see http://www.hoperf.com)
// RHDatagram will be received only by the addressed node or all nodes within range if the
// to address is RH_BROADCAST_ADDRESS
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RHMesh.cpp,v 1.11 2019/09/06 04:40:40 mikem Exp $

#include <RHMesh.h>

uint8_t RHMesh::_tmpMessage[RH_ROUTER_MAX_MESSAGE_LEN];

////////////////////////////////////////////////////////////////////
// Constructors
RHMesh::RHMesh(RHGenericDriver &driver, uint8_t thisAddress)
	: RHRouter(driver, thisAddress)
{
}

////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////
bool RHMesh::sendtoWaitAckEndToEnd(uint8_t *buf, uint8_t len, uint8_t dest, uint8_t flags)
{
	//Se haran 3 envios de mensaje a lo sumo
	int i;
	for (i = 0; i <= 2; i++)
	{
		Serial.print(F("Estoy por mandar un mensaje al nodo "));
		Serial.print(dest);
		Serial.print(F(" con la flag "));
		Serial.print(flags);
		Serial.print(F(" y el contenido "));
		Serial.println(*buf);

		//Si el mensaje no llega al próximo salto, entonces directamente volver a comenzar la iteración
		//La función sendToWait devuelve 0 si el mensaje es confirmado por el siguiente salto
		if (RHMesh::sendtoWait(buf, len, dest, flags))
		{
			Serial.print(F("En i= "));
			Serial.print(i);
			Serial.println(F(" el mensaje NO llegó exitosamente al siguiente salto"));

			continue;
		}

		Serial.print(F("En i= "));
		Serial.print(i);
		Serial.println(F(" el mensaje llegó exitosamente al siguiente salto"));

		uint8_t tmpMessageLen = sizeof(_tmpMessage);
		uint8_t _source;
		uint8_t _dest;
		uint8_t _id;
		uint8_t _flags;

		//Esperar 10 segundos a recibir el ACK del destino, caso contrario, volver a enviar el mensaje

		// _tmpMessage, &tmpMessageLen, &_source, &_dest, &_id, &_flags
		if (RHMesh::recvfromAckTimeout(_tmpMessage, &tmpMessageLen, 10000, &_source, &_dest, &_id, &_flags))
		{
			Serial.print(F("Recibi un mensaje en RHMesh::recvfromAckTimeout. Viene del nodo "));
			Serial.print(_source);
			Serial.print(" y el flag ");
			Serial.print(_flags);
			Serial.print(". El contenido es: ");
			Serial.print(_tmpMessage[0]);
			Serial.print(_tmpMessage[1]);
			Serial.print(_tmpMessage[2]);
			Serial.print(_tmpMessage[3]);
			Serial.print(_tmpMessage[4]);
			Serial.print(" y el largo de tmpMessageLen es ");
			Serial.println(tmpMessageLen);


			//Si el paquete tiene la flag de ACK response, viene desde el destino y tiene el mismo id
			if ((_flags & ROUTER_FLAGS_ACK_RESPONSE) && _source == dest )
			{
				return true;
			}
		}
	}

	return false;
}

// Discovers a route to the destination (if necessary), sends and
// waits for delivery to the next hop (but not for delivery to the final destination)
uint8_t RHMesh::sendtoWait(uint8_t *buf, uint8_t len, uint8_t address, uint8_t flags)
{
	//----------------------------------------------------------- AGREGADO POR MI
	Serial.print(F("Voy a intentar enviar un mensaje al nodo "));
	Serial.println(address);

	if (len > RH_MESH_MAX_MESSAGE_LEN)
		return RH_ROUTER_ERROR_INVALID_LENGTH;

	if (address != RH_BROADCAST_ADDRESS)
	{
		RoutingTableEntry *route = getRouteTo(address);

		//----------------------------------------------------------- AGREGADO POR MI
		if (!route)
		{
			Serial.println(F("---- No tengo la ruta guardada. Voy a intentar buscar una ruta "));
			Serial.print(F("---- Voy a enviar un broadcast con un DISCOVERY REQUEST con nodo de destino numero "));
			Serial.println(address);
		}

		if (!route && !doArp(address))
		{
			Serial.println(F("---- No encontré ruta"));

			return RH_ROUTER_ERROR_NO_ROUTE;
		}
	}

	// Now have a route. Contruct an application layer message and send it via that route
	MeshApplicationMessage *a = (MeshApplicationMessage *)&_tmpMessage;
	a->header.msgType = RH_MESH_MESSAGE_TYPE_APPLICATION;
	memcpy(a->data, buf, len);
	return RHRouter::sendtoWait(_tmpMessage, sizeof(RHMesh::MeshMessageHeader) + len, address, flags);
}

////////////////////////////////////////////////////////////////////
bool RHMesh::doArp(uint8_t address)
{
	// Need to discover a route
	// Broadcast a route discovery message with nothing in it
	MeshRouteDiscoveryMessage *p = (MeshRouteDiscoveryMessage *)&_tmpMessage;
	p->header.msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST;
	p->destlen = 1;
	p->dest = address; // Who we are looking for

	Serial.println(F("---- Estoy por enviar broadcast para encontrar ruta "));
	uint8_t error = RHRouter::sendtoWait((uint8_t *)p, sizeof(RHMesh::MeshMessageHeader) + 2, RH_BROADCAST_ADDRESS);
	if (error != RH_ROUTER_ERROR_NONE)
	{
		Serial.println(F("---- Hubo un error en el envio del broadcast "));

		return false;
	}

	Serial.println(F("---- El broadcast se envío correctamente"));

	// Wait for a reply, which will be unicast back to us
	// It will contain the complete route to the destination
	uint8_t messageLen = sizeof(_tmpMessage);
	// FIXME: timeout should be configurable
	unsigned long starttime = millis();
	int32_t timeLeft;
	while ((timeLeft = RH_MESH_ARP_TIMEOUT - (millis() - starttime)) > 0)
	{
		if (waitAvailableTimeout(timeLeft))
		{
			if (RHRouter::recvfromAck(_tmpMessage, &messageLen))
			{
				if (messageLen > 1 && p->header.msgType == RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE)
				{
					//----------------------------------------------------------- AGREGADO POR MI
					Serial.print(F("---- Encontré una ruta al nodo "));
					Serial.print(address);
					Serial.print(F(" y necesito ir por el nodo"));
					Serial.println(headerFrom());
					Serial.println(F("--------------------------------------------------------------"));

					// Got a reply, now add the next hop to the dest to the routing table
					// The first hop taken is the first octet
					addRouteTo(address, headerFrom());

					//----------------------------------------------------------- AGREGADO POR MI
					//para agregar la ruta del nodo que me envía el DISCOVERY RESPONSE (no el que lo originó, sino el último nodo que me lo envía)
					//addRouteTo(headerFrom(), headerFrom());

					return true;
				}
			}
		}
		YIELD;
	}
	return false;
}

////////////////////////////////////////////////////////////////////
// Called by RHRouter::recvfromAck whenever a message goes past
void RHMesh::peekAtMessage(RoutedMessage *message, uint8_t messageLen)
{
	MeshMessageHeader *m = (MeshMessageHeader *)message->data;
	if (messageLen > 1 && m->msgType == RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE)
	{
		//----------------------------------------------------------- AGREGADO POR MI
		Serial.print(F("Recibí un mensaje de DISCOVERY RESPONSE del nodo "));
		Serial.println(headerFrom());

		// This is a unicast RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE messages
		// being routed back to the originator here. Want to scrape some routing data out of the response
		// We can find the routes to all the nodes between here and the responding node
		MeshRouteDiscoveryMessage *d = (MeshRouteDiscoveryMessage *)message->data;
		addRouteTo(d->dest, headerFrom());
		uint8_t numRoutes = messageLen - sizeof(RoutedMessageHeader) - sizeof(MeshMessageHeader) - 2;
		uint8_t i;
		// Find us in the list of nodes that were traversed to get to the responding node
		for (i = 0; i < numRoutes; i++)
			if (d->route[i] == _thisAddress)
				break;
		i++;
		while (i < numRoutes)
			addRouteTo(d->route[i++], headerFrom());

		//----------------------------------------------------------- AGREGADO POR MI
		if (message->header.dest == _thisAddress)
		{
			Serial.println(F("El DISCOVERY RESPONSE era para mí"));

			for (i = 0; i < numRoutes; i++)
			{
				addRouteTo(d->route[i], headerFrom());
				// Serial.print(F("Voy a agregar a mi tabla al nodo "));
				// Serial.print(d->route[i]);
				// Serial.print(F(", cuyo next hop es "));
				// Serial.println(headerFrom());
			}
		}
		else
		{
			Serial.println(F("El DISCOVERY RESPONSE no era para mí"));
		}

		Serial.println(F("--------------------------------------------------------------"));
	}
	else if (messageLen > 1 && m->msgType == RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE)
	{
		MeshRouteFailureMessage *d = (MeshRouteFailureMessage *)message->data;

		//----------------------------------------------------------- AGREGADO POR MI
		Serial.print(F("Recibí un mensaje de ROUTE_FAILURE para el nodo "));
		Serial.println(d->dest);
		Serial.println(F("--------------------------------------------------------------"));

		deleteRouteTo(d->dest);
	}
}

////////////////////////////////////////////////////////////////////
// This is called when a message is to be delivered to the next hop
uint8_t RHMesh::route(RoutedMessage *message, uint8_t messageLen)
{
	uint8_t from = headerFrom(); // Might get clobbered during call to superclass route()
	uint8_t ret = RHRouter::route(message, messageLen);
	if (ret == RH_ROUTER_ERROR_NO_ROUTE || ret == RH_ROUTER_ERROR_UNABLE_TO_DELIVER)
	{
		// Cant deliver to the next hop. Delete the route
		deleteRouteTo(message->header.dest);
		if (message->header.source != _thisAddress)
		{
			// This is being proxied, so tell the originator about it
			MeshRouteFailureMessage *p = (MeshRouteFailureMessage *)&_tmpMessage;
			p->header.msgType = RH_MESH_MESSAGE_TYPE_ROUTE_FAILURE;
			p->dest = message->header.dest; // Who you were trying to deliver to
			// Make sure there is a route back towards whoever sent the original message
			addRouteTo(message->header.source, from);
			ret = RHRouter::sendtoWait((uint8_t *)p, sizeof(RHMesh::MeshMessageHeader) + 1, message->header.source);
		}
	}
	return ret;
}

////////////////////////////////////////////////////////////////////
// Subclasses may want to override
bool RHMesh::isPhysicalAddress(uint8_t *address, uint8_t addresslen)
{
	// Can only handle physical addresses 1 octet long, which is the physical node address
	return addresslen == 1 && address[0] == _thisAddress;
}

////////////////////////////////////////////////////////////////////
bool RHMesh::recvfromAck(uint8_t *buf, uint8_t *len, uint8_t *source, uint8_t *dest, uint8_t *id, uint8_t *flags)
{
	uint8_t tmpMessageLen = sizeof(_tmpMessage);
	uint8_t _source;
	uint8_t _dest;
	uint8_t _id;
	uint8_t _flags;
	if (RHRouter::recvfromAck(_tmpMessage, &tmpMessageLen, &_source, &_dest, &_id, &_flags))
	{

		if (_flags & RH_FLAG_MOVIL)
		{
			if (source)
				*source = _source;
			if (dest)
				*dest = _dest;
			if (id)
				*id = _id;
			if (flags)
				*flags = _flags;
			uint8_t msgLen = tmpMessageLen;
			if (*len > msgLen)
				*len = msgLen;
			memcpy(buf, _tmpMessage, *len);

			return true;
		}

		MeshMessageHeader *p = (MeshMessageHeader *)&_tmpMessage;

		if (tmpMessageLen >= 1 && p->msgType == RH_MESH_MESSAGE_TYPE_APPLICATION)
		{
			MeshApplicationMessage *a = (MeshApplicationMessage *)p;
			// Handle application layer messages, presumably for our caller
			if (source)
				*source = _source;
			if (dest)
				*dest = _dest;
			if (id)
				*id = _id;
			if (flags)
				*flags = _flags;
			uint8_t msgLen = tmpMessageLen - sizeof(MeshMessageHeader);
			if (*len > msgLen)
				*len = msgLen;
			memcpy(buf, a->data, *len);

			Serial.print(F("************Estamos adentro del IF. El valor del mensaje es: "));
			Serial.println((char *)a->data);
			Serial.print(F("****-----***_tmpMessage vale: "));
			Serial.println((char *)_tmpMessage);
			Serial.print(F("****-----***&_tmpMessage vale: "));
			Serial.println((uint32_t )&_tmpMessage);
			Serial.print(F("****-----***p vale: "));
			Serial.println((char *)p);
			Serial.print(F("****-----***a vale: "));
			Serial.println((uint32_t )a);

			//TOMAS
			//Si pide un ACK entonces hay que enviar mensaje al nodo que originó el mensaje con flag de ROUTER_FLAGS_ACK_RESPONSE
			if (_flags & ROUTER_FLAGS_ACK_PETITION)
			{
				Serial.print(F("Se ha recibido un mensaje que requiere un ACK de extremo a extremo. El AP origen es: "));
				Serial.println(_source);

				if (!RHMesh::sendtoWait((uint8_t*) "ACKtk", 6, _source, ROUTER_FLAGS_ACK_RESPONSE))
				{
					Serial.print(F("Se ha enviado el ACK de extremo a extremo con flag "));
					Serial.println(ROUTER_FLAGS_ACK_RESPONSE);
				}
			}

			// Serial.println(F("En RHMesh el paquete es: "));
			// Serial.println((char[60])buf);
			// Serial.println(F("Y su tamaño es: "));
			// Serial.println(*len);

			return true;
		}
		else if (_dest == RH_BROADCAST_ADDRESS && tmpMessageLen > 1 && p->msgType == RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST)
		{

			//----------------------------------------------------------- AGREGADO POR MI
			Serial.println(F("El tipo de mensaje es de DISCOVERY_REQUEST"));

			MeshRouteDiscoveryMessage *d = (MeshRouteDiscoveryMessage *)p;
			// Handle Route discovery requests
			// Message is an array of node addresses the route request has already passed through
			// If it originally came from us, ignore it
			if (_source == _thisAddress)
				return false;

			uint8_t numRoutes = tmpMessageLen - sizeof(MeshMessageHeader) - 2;
			uint8_t i;
			// Are we already mentioned?
			for (i = 0; i < numRoutes; i++)
				if (d->route[i] == _thisAddress)
					return false; // Already been through us. Discard

			//----------------------------------------------------------- AGREGADO POR MI
			// Serial.print(F("Los nodos en el array del DISCOVERY REQUEST son: ("));
			// for (i = 0; i < numRoutes; i++)
			// {
			// 	if (i != numRoutes - 1)
			// 	{
			// 		Serial.print(d->route[i]);
			// 		Serial.print(F(", "));
			// 	}
			// 	else
			// 	{
			// 		Serial.print(d->route[i]);
			// 	}
			// }
			// Serial.println(F(")"));
			// Serial.println(F("--------------------------------------------------------------"));

			addRouteTo(_source, headerFrom()); // The originator needs to be added regardless of node type

			// Hasnt been past us yet, record routes back to the earlier nodes
			// No need to waste memory if we are not participating in routing
			if (_isa_router)
			{
				for (i = 0; i < numRoutes; i++)
					addRouteTo(d->route[i], headerFrom());
			}

			if (isPhysicalAddress(&d->dest, d->destlen))
			{
				//----------------------------------------------------------- AGREGADO POR MI
				Serial.println(F("El DISCOVERY REQUEST era para mí"));
				Serial.println(F("Voy a responder con un DISCOVERY RESPONSE"));
				Serial.println(F("--------------------------------------------------------------"));

				// This route discovery is for us. Unicast the whole route back to the originator
				// as a RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE
				// We are certain to have a route there, because we just got it
				d->header.msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE;

				RHRouter::sendtoWait((uint8_t *)d, tmpMessageLen, _source);
			}
			else if ((i < _max_hops) && _isa_router)
			{
				//----------------------------------------------------------- AGREGADO POR MI
				Serial.println(F("El DISCOVERY REQUEST NO era para mí"));
				Serial.println(F("Voy a agregarme al array de rutas y a enviar otro broadcast"));
				Serial.println(F("--------------------------------------------------------------"));

				// Its for someone else, rebroadcast it, after adding ourselves to the list
				d->route[numRoutes] = _thisAddress;
				tmpMessageLen++;
				// Have to impersonate the source
				// REVISIT: if this fails what can we do?
				RHRouter::sendtoFromSourceWait(_tmpMessage, tmpMessageLen, RH_BROADCAST_ADDRESS, _source);
			}
		}
	}
	return false;
}

////////////////////////////////////////////////////////////////////
bool RHMesh::recvfromAckTimeout(uint8_t *buf, uint8_t *len, uint16_t timeout, uint8_t *from, uint8_t *to, uint8_t *id, uint8_t *flags)
{
	unsigned long starttime = millis();
	int32_t timeLeft;
	while ((timeLeft = timeout - (millis() - starttime)) > 0)
	{
		if (waitAvailableTimeout(timeLeft))
		{
			if (recvfromAck(buf, len, from, to, id, flags))
				return true;
			YIELD;
		}
	}
	return false;
}
