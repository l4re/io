#include "dma_domain.h"
#include "debug.h"
#include <cassert>
#include <cstdio>
#include <l4/re/env>
#include <l4/re/error_helper>

unsigned Dma_domain::_next_free_domain;
bool Dma_domain_if::_supports_remapping;

void
Dma_domain::add_to_group(Dma_domain_group *g)
{
  if (!_v_domain && !g->_set)
    {
      Dma_domain_set *n = new Dma_domain_set();
      g->assign_set(n);
      add_to_set(n);
      return;
    }

  if (_v_domain == g->_set)
    return;

  if (_v_domain && g->_set)
    {
      // domain is already in a group and the target group already contains
      // other domains
      g->merge(_v_domain);
      return;
    }

  if (_v_domain)
    {
      g->assign_set(_v_domain);
      return;
    }

  add_to_set(g->_set);
}


Managed_dma_space::Managed_dma_space(L4Re::Util::Unique_cap<L4Re::Dma_space> dma_space)
: _dma_space(std::move(dma_space)),
  _dma_task(L4Re::chkcap(L4Re::Util::make_unique_cap<L4::Task>()))
{
  L4Re::chksys(L4Re::Env::env()->factory()
                  ->create(_dma_task.get(), L4_PROTO_DMA_SPACE));
}

Managed_dma_space::~Managed_dma_space()
{
  auto dma_mgr =
    L4Re::chkcap(L4Re::Env::env()->get_cap<L4Re::Dma_space_mgr>("dma_mgr"),
                 "Get DMA space manager cap from env");

  int ret = dma_mgr->disassociate(L4::Ipc::make_cap_rws(_dma_space.get()));
  if (ret < 0)
    d_printf(DBG_ERR, "Could not disassociate: %d\n", ret);
}


int
Dma_domain_if::set_dma_space(L4Re::Util::Unique_cap<L4Re::Dma_space> dma_space)
{
  auto dma_mgr =
    L4Re::chkcap(L4Re::Env::env()->get_cap<L4Re::Dma_space_mgr>("dma_mgr"),
                 "Get DMA space manager cap from env");

  if (!_supports_remapping)
    {
      d_printf(DBG_DEBUG2, "DMA: use CPU-phys addresses for DMA\n");
      return dma_mgr->associate_phys(L4::Ipc::make_cap_rws(dma_space.get()),
                                     L4Re::Dma_space_mgr::Space_attribs::None);
    }

  d_printf(DBG_DEBUG2, "DMA: create kern DMA space for managed DMA\n");
  auto mds = std::make_shared<Managed_dma_space>(std::move(dma_space));

  // Bind the device / all devices to the Managed_dma_space.
  if (int err = set_managed_dma_space(mds); err < 0)
    return err;

  d_printf(DBG_DEBUG2, "DMA: associate managed DMA space (cap=%lx)\n",
           mds->dma_task().cap());
  int err = dma_mgr->associate(L4::Ipc::make_cap_rws(mds->dma_space()),
                               L4::Ipc::make_cap_rws(mds->dma_task()),
                               L4Re::Dma_space_mgr::Space_attribs::None);
  if (err < 0)
    {
      d_printf(DBG_DEBUG2, "DMA: associate failed: %d\n", err);
      clear_managed_dma_space();
    }

  return err;
}

int
Dma_domain_if::clear_dma_space()
{
  if (!_supports_remapping || !_managed_dma_space)
    return 0;

  // This will unbind the device(s) from the IOMMU.
  clear_managed_dma_space();

  return 0;
}

int
Dma_domain_if::set_managed_dma_space(std::shared_ptr<Managed_dma_space> space)
{
  if (_managed_dma_space)
    return -L4_EBUSY;

  _managed_dma_space = space;
  return 0;
}

void
Dma_domain_if::clear_managed_dma_space()
{ _managed_dma_space = nullptr; }


int
Dma_domain_set::set_managed_dma_space(std::shared_ptr<Managed_dma_space> space)
{
  int i, ret = 0;

  for (i = 0; i < static_cast<int>(_domains.size()); i++)
    if ((ret = _domains[i]->set_managed_dma_space(space)) < 0)
      break;

  if (ret < 0)
    while (i >= 0)
      _domains[i--]->clear_managed_dma_space();

  return ret;
}

void
Dma_domain_set::clear_managed_dma_space()
{
  for (auto d : _domains)
    d->clear_managed_dma_space();
}
