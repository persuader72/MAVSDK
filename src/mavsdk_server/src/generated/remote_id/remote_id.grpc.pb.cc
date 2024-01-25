// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: remote_id/remote_id.proto

#include "remote_id/remote_id.pb.h"
#include "remote_id/remote_id.grpc.pb.h"

#include <functional>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/impl/channel_interface.h>
#include <grpcpp/impl/client_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/rpc_service_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/sync_stream.h>
namespace mavsdk {
namespace rpc {
namespace remote_id {

static const char* RemoteIdService_method_names[] = {
  "/mavsdk.rpc.remote_id.RemoteIdService/SetBasicId",
  "/mavsdk.rpc.remote_id.RemoteIdService/SetLocation",
  "/mavsdk.rpc.remote_id.RemoteIdService/SetLocationAccuracy",
  "/mavsdk.rpc.remote_id.RemoteIdService/SetSystem",
  "/mavsdk.rpc.remote_id.RemoteIdService/SetOperatorId",
  "/mavsdk.rpc.remote_id.RemoteIdService/SetSelfId",
  "/mavsdk.rpc.remote_id.RemoteIdService/SubscribeArmStatus",
};

std::unique_ptr< RemoteIdService::Stub> RemoteIdService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< RemoteIdService::Stub> stub(new RemoteIdService::Stub(channel, options));
  return stub;
}

RemoteIdService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options)
  : channel_(channel), rpcmethod_SetBasicId_(RemoteIdService_method_names[0], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SetLocation_(RemoteIdService_method_names[1], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SetLocationAccuracy_(RemoteIdService_method_names[2], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SetSystem_(RemoteIdService_method_names[3], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SetOperatorId_(RemoteIdService_method_names[4], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SetSelfId_(RemoteIdService_method_names[5], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SubscribeArmStatus_(RemoteIdService_method_names[6], options.suffix_for_stats(),::grpc::internal::RpcMethod::SERVER_STREAMING, channel)
  {}

::grpc::Status RemoteIdService::Stub::SetBasicId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetBasicIdRequest& request, ::mavsdk::rpc::remote_id::SetBasicIdResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::remote_id::SetBasicIdRequest, ::mavsdk::rpc::remote_id::SetBasicIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetBasicId_, context, request, response);
}

void RemoteIdService::Stub::async::SetBasicId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetBasicIdRequest* request, ::mavsdk::rpc::remote_id::SetBasicIdResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::remote_id::SetBasicIdRequest, ::mavsdk::rpc::remote_id::SetBasicIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetBasicId_, context, request, response, std::move(f));
}

void RemoteIdService::Stub::async::SetBasicId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetBasicIdRequest* request, ::mavsdk::rpc::remote_id::SetBasicIdResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetBasicId_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetBasicIdResponse>* RemoteIdService::Stub::PrepareAsyncSetBasicIdRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetBasicIdRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::remote_id::SetBasicIdResponse, ::mavsdk::rpc::remote_id::SetBasicIdRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetBasicId_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetBasicIdResponse>* RemoteIdService::Stub::AsyncSetBasicIdRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetBasicIdRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetBasicIdRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status RemoteIdService::Stub::SetLocation(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationRequest& request, ::mavsdk::rpc::remote_id::SetLocationResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::remote_id::SetLocationRequest, ::mavsdk::rpc::remote_id::SetLocationResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetLocation_, context, request, response);
}

void RemoteIdService::Stub::async::SetLocation(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationRequest* request, ::mavsdk::rpc::remote_id::SetLocationResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::remote_id::SetLocationRequest, ::mavsdk::rpc::remote_id::SetLocationResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetLocation_, context, request, response, std::move(f));
}

void RemoteIdService::Stub::async::SetLocation(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationRequest* request, ::mavsdk::rpc::remote_id::SetLocationResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetLocation_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetLocationResponse>* RemoteIdService::Stub::PrepareAsyncSetLocationRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::remote_id::SetLocationResponse, ::mavsdk::rpc::remote_id::SetLocationRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetLocation_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetLocationResponse>* RemoteIdService::Stub::AsyncSetLocationRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetLocationRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status RemoteIdService::Stub::SetLocationAccuracy(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest& request, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetLocationAccuracy_, context, request, response);
}

void RemoteIdService::Stub::async::SetLocationAccuracy(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest* request, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetLocationAccuracy_, context, request, response, std::move(f));
}

void RemoteIdService::Stub::async::SetLocationAccuracy(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest* request, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetLocationAccuracy_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse>* RemoteIdService::Stub::PrepareAsyncSetLocationAccuracyRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse, ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetLocationAccuracy_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse>* RemoteIdService::Stub::AsyncSetLocationAccuracyRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetLocationAccuracyRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status RemoteIdService::Stub::SetSystem(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSystemRequest& request, ::mavsdk::rpc::remote_id::SetSystemResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::remote_id::SetSystemRequest, ::mavsdk::rpc::remote_id::SetSystemResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetSystem_, context, request, response);
}

void RemoteIdService::Stub::async::SetSystem(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSystemRequest* request, ::mavsdk::rpc::remote_id::SetSystemResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::remote_id::SetSystemRequest, ::mavsdk::rpc::remote_id::SetSystemResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetSystem_, context, request, response, std::move(f));
}

void RemoteIdService::Stub::async::SetSystem(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSystemRequest* request, ::mavsdk::rpc::remote_id::SetSystemResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetSystem_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetSystemResponse>* RemoteIdService::Stub::PrepareAsyncSetSystemRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSystemRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::remote_id::SetSystemResponse, ::mavsdk::rpc::remote_id::SetSystemRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetSystem_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetSystemResponse>* RemoteIdService::Stub::AsyncSetSystemRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSystemRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetSystemRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status RemoteIdService::Stub::SetOperatorId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetOperatorIdRequest& request, ::mavsdk::rpc::remote_id::SetOperatorIdResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::remote_id::SetOperatorIdRequest, ::mavsdk::rpc::remote_id::SetOperatorIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetOperatorId_, context, request, response);
}

void RemoteIdService::Stub::async::SetOperatorId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetOperatorIdRequest* request, ::mavsdk::rpc::remote_id::SetOperatorIdResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::remote_id::SetOperatorIdRequest, ::mavsdk::rpc::remote_id::SetOperatorIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetOperatorId_, context, request, response, std::move(f));
}

void RemoteIdService::Stub::async::SetOperatorId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetOperatorIdRequest* request, ::mavsdk::rpc::remote_id::SetOperatorIdResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetOperatorId_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetOperatorIdResponse>* RemoteIdService::Stub::PrepareAsyncSetOperatorIdRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetOperatorIdRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::remote_id::SetOperatorIdResponse, ::mavsdk::rpc::remote_id::SetOperatorIdRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetOperatorId_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetOperatorIdResponse>* RemoteIdService::Stub::AsyncSetOperatorIdRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetOperatorIdRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetOperatorIdRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status RemoteIdService::Stub::SetSelfId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSelfIdRequest& request, ::mavsdk::rpc::remote_id::SetSelfIdResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::remote_id::SetSelfIdRequest, ::mavsdk::rpc::remote_id::SetSelfIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetSelfId_, context, request, response);
}

void RemoteIdService::Stub::async::SetSelfId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSelfIdRequest* request, ::mavsdk::rpc::remote_id::SetSelfIdResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::remote_id::SetSelfIdRequest, ::mavsdk::rpc::remote_id::SetSelfIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetSelfId_, context, request, response, std::move(f));
}

void RemoteIdService::Stub::async::SetSelfId(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSelfIdRequest* request, ::mavsdk::rpc::remote_id::SetSelfIdResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetSelfId_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetSelfIdResponse>* RemoteIdService::Stub::PrepareAsyncSetSelfIdRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSelfIdRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::remote_id::SetSelfIdResponse, ::mavsdk::rpc::remote_id::SetSelfIdRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetSelfId_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::remote_id::SetSelfIdResponse>* RemoteIdService::Stub::AsyncSetSelfIdRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SetSelfIdRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetSelfIdRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::ClientReader< ::mavsdk::rpc::remote_id::ArmStatusResponse>* RemoteIdService::Stub::SubscribeArmStatusRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest& request) {
  return ::grpc::internal::ClientReaderFactory< ::mavsdk::rpc::remote_id::ArmStatusResponse>::Create(channel_.get(), rpcmethod_SubscribeArmStatus_, context, request);
}

void RemoteIdService::Stub::async::SubscribeArmStatus(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest* request, ::grpc::ClientReadReactor< ::mavsdk::rpc::remote_id::ArmStatusResponse>* reactor) {
  ::grpc::internal::ClientCallbackReaderFactory< ::mavsdk::rpc::remote_id::ArmStatusResponse>::Create(stub_->channel_.get(), stub_->rpcmethod_SubscribeArmStatus_, context, request, reactor);
}

::grpc::ClientAsyncReader< ::mavsdk::rpc::remote_id::ArmStatusResponse>* RemoteIdService::Stub::AsyncSubscribeArmStatusRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
  return ::grpc::internal::ClientAsyncReaderFactory< ::mavsdk::rpc::remote_id::ArmStatusResponse>::Create(channel_.get(), cq, rpcmethod_SubscribeArmStatus_, context, request, true, tag);
}

::grpc::ClientAsyncReader< ::mavsdk::rpc::remote_id::ArmStatusResponse>* RemoteIdService::Stub::PrepareAsyncSubscribeArmStatusRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncReaderFactory< ::mavsdk::rpc::remote_id::ArmStatusResponse>::Create(channel_.get(), cq, rpcmethod_SubscribeArmStatus_, context, request, false, nullptr);
}

RemoteIdService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SetBasicIdRequest, ::mavsdk::rpc::remote_id::SetBasicIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SetBasicIdRequest* req,
             ::mavsdk::rpc::remote_id::SetBasicIdResponse* resp) {
               return service->SetBasicId(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SetLocationRequest, ::mavsdk::rpc::remote_id::SetLocationResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SetLocationRequest* req,
             ::mavsdk::rpc::remote_id::SetLocationResponse* resp) {
               return service->SetLocation(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest* req,
             ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse* resp) {
               return service->SetLocationAccuracy(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[3],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SetSystemRequest, ::mavsdk::rpc::remote_id::SetSystemResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SetSystemRequest* req,
             ::mavsdk::rpc::remote_id::SetSystemResponse* resp) {
               return service->SetSystem(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[4],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SetOperatorIdRequest, ::mavsdk::rpc::remote_id::SetOperatorIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SetOperatorIdRequest* req,
             ::mavsdk::rpc::remote_id::SetOperatorIdResponse* resp) {
               return service->SetOperatorId(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[5],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SetSelfIdRequest, ::mavsdk::rpc::remote_id::SetSelfIdResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SetSelfIdRequest* req,
             ::mavsdk::rpc::remote_id::SetSelfIdResponse* resp) {
               return service->SetSelfId(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RemoteIdService_method_names[6],
      ::grpc::internal::RpcMethod::SERVER_STREAMING,
      new ::grpc::internal::ServerStreamingHandler< RemoteIdService::Service, ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest, ::mavsdk::rpc::remote_id::ArmStatusResponse>(
          [](RemoteIdService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest* req,
             ::grpc::ServerWriter<::mavsdk::rpc::remote_id::ArmStatusResponse>* writer) {
               return service->SubscribeArmStatus(ctx, req, writer);
             }, this)));
}

RemoteIdService::Service::~Service() {
}

::grpc::Status RemoteIdService::Service::SetBasicId(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SetBasicIdRequest* request, ::mavsdk::rpc::remote_id::SetBasicIdResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RemoteIdService::Service::SetLocation(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SetLocationRequest* request, ::mavsdk::rpc::remote_id::SetLocationResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RemoteIdService::Service::SetLocationAccuracy(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SetLocationAccuracyRequest* request, ::mavsdk::rpc::remote_id::SetLocationAccuracyResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RemoteIdService::Service::SetSystem(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SetSystemRequest* request, ::mavsdk::rpc::remote_id::SetSystemResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RemoteIdService::Service::SetOperatorId(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SetOperatorIdRequest* request, ::mavsdk::rpc::remote_id::SetOperatorIdResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RemoteIdService::Service::SetSelfId(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SetSelfIdRequest* request, ::mavsdk::rpc::remote_id::SetSelfIdResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RemoteIdService::Service::SubscribeArmStatus(::grpc::ServerContext* context, const ::mavsdk::rpc::remote_id::SubscribeArmStatusRequest* request, ::grpc::ServerWriter< ::mavsdk::rpc::remote_id::ArmStatusResponse>* writer) {
  (void) context;
  (void) request;
  (void) writer;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace mavsdk
}  // namespace rpc
}  // namespace remote_id

